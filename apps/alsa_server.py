#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ALSA mixer server.

Purpose:
  - Expose ALSA mixer controls (e.g. DSPVolume) to Home Assistant via MQTT Discovery.
  - Two-way sync:
      * HA -> MQTT command_topic -> amixer sset
      * alsamixer/other -> alsactl monitor / polling -> MQTT state_topic -> HA

Conventions:
  - Single config.yaml for all apps.
  - MQTT Discovery (retained config + state).
  - Subscribe to homeassistant/status and republish discovery on HA restart.
"""

import json
import re
import signal
import subprocess
import sys
import time
from threading import Event, Thread
from typing import Any, Dict, Optional, Set

import paho.mqtt.client as mqtt

from mqtt_gpio.config import load_config
from mqtt_gpio.logging_utils import setup_rotating_logger
from mqtt_gpio.mqtt_base import make_client
from mqtt_gpio.safety import RateLimiter
from mqtt_gpio.utils import to_object_id


CFG = load_config()

# MQTT
MQTT_HOST = CFG["mqtt"]["host"]
MQTT_PORT = int(CFG["mqtt"]["port"])
MQTT_USER = CFG["mqtt"].get("username")
MQTT_PW = CFG["mqtt"].get("password")
BASE = CFG["mqtt"]["base_topic"].rstrip("/")
DISCOVERY = CFG["mqtt"]["discovery_prefix"].rstrip("/")

# Device
DEVICE_ID = CFG["device"]["id"]
DEVICE_NAME = CFG["device"]["name"]
DEVICE_MANU = CFG["device"]["manufacturer"]
DEVICE_MODEL = CFG["device"]["model"]
SW_VERSION = CFG["device"]["sw_version"]

# Logging
LOG = setup_rotating_logger("alsa_server", CFG.get("logging", {}))

# Safety defaults
SCFG = CFG.get("safety", {})
DEFAULT_MIN_CMD_INTERVAL_SEC = float(SCFG.get("min_command_interval_sec", 0.2))

# ALSA config
ALSA_SECTION: Dict[str, Any] = CFG.get("alsa", {}) or {}
POLL_SEC = float(ALSA_SECTION.get("poll_sec", 1.0))
ENABLE_MONITOR = bool(ALSA_SECTION.get("enable_monitor", True))
ALSA_MIN_CMD_INTERVAL_SEC = float(ALSA_SECTION.get("min_command_interval_sec", DEFAULT_MIN_CMD_INTERVAL_SEC))

_GLOBAL_KEYS = {"poll_sec", "enable_monitor", "min_command_interval_sec", "controls"}
CONTROLS_CFG = ALSA_SECTION.get("controls")
if CONTROLS_CFG is None:
    # Backward-compatible format: alsa: { <object_id>: {..}, ... }
    CONTROLS_CFG = {k: v for k, v in ALSA_SECTION.items() if k not in _GLOBAL_KEYS}

if not isinstance(CONTROLS_CFG, dict) or not CONTROLS_CFG:
    raise SystemExit("Config section 'alsa.controls' is empty.")

# Availability
AVAIL_TOPIC = f"{BASE}/alsa_devices/{DEVICE_ID}/status"
PAYLOAD_ONLINE = "online"
PAYLOAD_OFFLINE = "offline"

# Regex for parsing percent from amixer output
_PCT_RE = re.compile(r"\[(\d+)%\]")


def device_obj() -> Dict[str, Any]:
    return {
        "identifiers": [DEVICE_ID],
        "name": DEVICE_NAME,
        "manufacturer": DEVICE_MANU,
        "model": DEVICE_MODEL,
        "sw_version": SW_VERSION,
    }


def _state_topic(oid: str) -> str:
    return f"{BASE}/alsa/{oid}/state"


def _cmd_topic(oid: str) -> str:
    return f"{BASE}/alsa/{oid}/set"


def _disc_topic(oid: str) -> str:
    return f"{DISCOVERY}/number/{oid}/config"


def _run(cmd: list[str], timeout: float = 2.0) -> subprocess.CompletedProcess:
    """Run a command safely and return CompletedProcess."""
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def _read_pct(card: int, control: str) -> Optional[int]:
    """Read mixer control percent (0..100). Returns None on parse/command error."""
    try:
        res = _run(["amixer", "-c", str(card), "sget", control], timeout=2.0)
        if res.returncode != 0:
            LOG.warning(f"amixer sget failed (card={card}, control={control}): rc={res.returncode}")
            return None
        m = _PCT_RE.search(res.stdout)
        if not m:
            return None
        return int(m.group(1))
    except FileNotFoundError:
        LOG.error("amixer not found. Install 'alsa-utils'.")
        return None
    except Exception as e:
        LOG.warning(f"amixer sget exception (card={card}, control={control}): {e}")
        return None


def _set_pct(card: int, control: str, pct: int) -> bool:
    """Set mixer control percent (0..100). Returns True if command succeeded."""
    try:
        res = _run(["amixer", "-c", str(card), "sset", control, f"{pct}%"], timeout=2.0)
        if res.returncode != 0:
            LOG.warning(f"amixer sset failed (card={card}, control={control}): rc={res.returncode}")
            return False
        return True
    except FileNotFoundError:
        LOG.error("amixer not found. Install 'alsa-utils'.")
        return False
    except Exception as e:
        LOG.warning(f"amixer sset exception (card={card}, control={control}): {e}")
        return False


def _parse_set_payload(payload_bytes: bytes) -> Optional[int]:
    """Parse MQTT payload for setting a number.

    Supports:
      - plain numeric string: "42" / "42.0"
      - JSON: {"value": 42}
    """
    s = payload_bytes.decode("utf-8", errors="ignore").strip()
    if not s:
        return None
    try:
        if s.startswith("{"):
            obj = json.loads(s)
            if "value" not in obj:
                return None
            return int(float(obj["value"]))
        return int(float(s))
    except Exception:
        return None


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(v)))


def _ensure_controls(cfg: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """Normalize controls config to internal structure."""
    out: Dict[str, Dict[str, Any]] = {}
    for key, meta in cfg.items():
        oid = to_object_id(key)
        if not isinstance(meta, dict):
            meta = {}

        card = int(meta.get("card", 0))
        control = str(meta.get("control", key))
        mn = int(meta.get("min", 0))
        mx = int(meta.get("max", 100))
        step = int(meta.get("step", 1))
        unit = str(meta.get("unit", "%"))
        icon = meta.get("icon")
        min_cmd = float(meta.get("min_command_interval_sec", ALSA_MIN_CMD_INTERVAL_SEC))

        out[oid] = {
            "name": meta.get("name", key),
            "card": card,
            "control": control,
            "min": mn,
            "max": mx,
            "step": step,
            "unit": unit,
            "icon": icon,
            "rate": RateLimiter(min_command_interval_sec=min_cmd),
            "last_pub": None,
        }
    return out


CONTROLS = _ensure_controls(CONTROLS_CFG)

# MQTT
client: mqtt.Client = make_client(AVAIL_TOPIC, PAYLOAD_OFFLINE, MQTT_USER, MQTT_PW)

# Wake-up event to trigger a poll outside of the normal polling cadence
_wake_event = Event()
_stop_event = Event()


def _publish_state(oid: str, force: bool = False) -> None:
    m = CONTROLS[oid]
    pct = _read_pct(int(m["card"]), str(m["control"]))
    if pct is None:
        return
    pct = _clamp(pct, int(m["min"]), int(m["max"]))
    if (not force) and (m.get("last_pub") == pct):
        return
    m["last_pub"] = pct
    client.publish(_state_topic(oid), str(pct), retain=True)
    LOG.debug(f"Published {oid}={pct}% -> {_state_topic(oid)}")


def publish_discovery() -> None:
    dev = device_obj()
    for oid, m in CONTROLS.items():
        payload: Dict[str, Any] = {
            "name": m["name"],
            "unique_id": f"{DEVICE_ID}_alsa_{oid}",
            "state_topic": _state_topic(oid),
            "command_topic": _cmd_topic(oid),
            "availability_topic": AVAIL_TOPIC,
            "payload_available": PAYLOAD_ONLINE,
            "payload_not_available": PAYLOAD_OFFLINE,
            "min": int(m["min"]),
            "max": int(m["max"]),
            "step": int(m["step"]),
            "unit_of_measurement": m["unit"],
            "mode": "slider",
            "device": dev,
        }
        if m.get("icon"):
            payload["icon"] = m["icon"]

        client.publish(_disc_topic(oid), json.dumps(payload), retain=True)
        LOG.debug(f"Discovery published for {oid} -> {_disc_topic(oid)}")

    for oid in CONTROLS:
        _publish_state(oid, force=True)


def _alsa_monitor_loop(card: int) -> None:
    """Monitor ALSA events for a given card and wake the poll loop."""
    backoff_sec = 2.0
    while not _stop_event.is_set():
        proc: Optional[subprocess.Popen] = None
        try:
            proc = subprocess.Popen(
                ["alsactl", "monitor", str(card)],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
            LOG.info(f"ALSA monitor started (card={card})")
            assert proc.stdout is not None
            for _line in proc.stdout:
                if _stop_event.is_set():
                    break
                # Any mixer event triggers a state refresh.
                _wake_event.set()
            # If monitor exits, restart with backoff.
            rc = proc.wait(timeout=1.0)
            LOG.warning(f"ALSA monitor exited (card={card}, rc={rc}), restarting")
        except FileNotFoundError:
            LOG.warning("alsactl not found. Install 'alsa-utils' or disable monitor.")
            return
        except Exception as e:
            LOG.warning(f"ALSA monitor error (card={card}): {e}")
        finally:
            if proc is not None:
                try:
                    proc.terminate()
                except Exception:
                    pass
        time.sleep(backoff_sec)


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        LOG.info("MQTT connected")
    else:
        LOG.error(f"MQTT connect failed rc={rc}")
    client.publish(AVAIL_TOPIC, PAYLOAD_ONLINE, retain=True)
    client.subscribe("homeassistant/status")
    for oid in CONTROLS:
        client.subscribe(_cmd_topic(oid))
    publish_discovery()


def on_message(client, userdata, msg):
    try:
        payload = msg.payload
        topic = msg.topic

        if topic == "homeassistant/status":
            s = payload.decode("utf-8", errors="ignore").strip().lower()
            if s == "online":
                LOG.info("HA online -> republish discovery")
                publish_discovery()
            return

        parts = topic.split("/")
        if len(parts) >= 4 and parts[-3] == "alsa" and parts[-1] == "set":
            oid = parts[-2]
            if oid not in CONTROLS:
                return
            m = CONTROLS[oid]

            v = _parse_set_payload(payload)
            if v is None:
                LOG.warning(f"Invalid payload for {oid}: {payload!r}")
                return

            v = _clamp(int(v), int(m["min"]), int(m["max"]))

            # Rate limit calls to amixer when slider is dragged.
            if not m["rate"].allow():
                LOG.debug(f"Rate-limited set for {oid}={v}")
                _wake_event.set()
                return

            ok = _set_pct(int(m["card"]), str(m["control"]), v)
            if ok:
                LOG.info(f"Set {oid} -> {v}%")
                _publish_state(oid, force=True)
            else:
                LOG.warning(f"Set failed {oid} -> {v}%")
                _wake_event.set()
    except Exception as e:
        LOG.exception(f"on_message error: {e}")


def cleanup_and_exit(code: int = 0):
    _stop_event.set()
    try:
        client.publish(AVAIL_TOPIC, PAYLOAD_OFFLINE, retain=True)
    except Exception:
        pass
    try:
        client.loop_stop()
        client.disconnect()
    except Exception:
        pass
    sys.exit(code)


def handle_sigterm(signum, frame):
    LOG.info("SIGTERM received, shutting down")
    cleanup_and_exit(0)


signal.signal(signal.SIGTERM, handle_sigterm)
signal.signal(signal.SIGINT, handle_sigterm)


def main():
    client.on_connect = on_connect
    client.on_message = on_message

    # Start ALSA monitor threads (one per unique card).
    if ENABLE_MONITOR:
        cards: Set[int] = {int(m["card"]) for m in CONTROLS.values()}
        for c in sorted(cards):
            Thread(target=_alsa_monitor_loop, args=(c,), daemon=True).start()
    else:
        LOG.info("ALSA monitor disabled (polling only)")

    client.connect(MQTT_HOST, MQTT_PORT, 30)
    LOG.info("alsa server started (HA discovery, availability, bidirectional sync)")
    client.loop_start()

    try:
        while True:
            # Poll periodically, but also wake up on ALSA monitor events.
            _wake_event.wait(timeout=max(0.1, POLL_SEC))
            _wake_event.clear()
            for oid in CONTROLS:
                _publish_state(oid, force=False)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_and_exit(0)


if __name__ == "__main__":
    main()
