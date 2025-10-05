#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Shutter server:
- time-based position estimation
- HA discovery, availability, persistence
- safety guards (flap/rate limiting)
- travel-time learning (EMA with clamping)
"""

import os
import sys
import time
import signal
import json
from typing import Dict, Any
from time import monotonic as now

import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt

from mqtt_gpio.config import load_config
from mqtt_gpio.logging_utils import setup_rotating_logger
from mqtt_gpio.utils import to_object_id
from mqtt_gpio.gpio_out import ensure_board_mode, setup_output, write, safe_cleanup_low
from mqtt_gpio.mqtt_base import make_client
from mqtt_gpio.safety import FlapGuard, RateLimiter, apply_state_tristate

CFG = load_config()

# MQTT
MQTT_HOST = CFG["mqtt"]["host"]
MQTT_PORT = int(CFG["mqtt"]["port"])
MQTT_USER = CFG["mqtt"].get("username")
MQTT_PW   = CFG["mqtt"].get("password")
BASE      = CFG["mqtt"]["base_topic"]
DISCOVERY = CFG["mqtt"]["discovery_prefix"]

# Device
DEVICE_ID    = CFG["device"]["id"]
DEVICE_NAME  = CFG["device"]["name"]
DEVICE_MANU  = CFG["device"]["manufacturer"]
DEVICE_MODEL = CFG["device"]["model"]
SW_VERSION   = CFG["device"]["sw_version"]

# Logging
LOG = setup_rotating_logger("server_shutter", CFG.get("logging", {}))

# Safety defaults
SCFG = CFG.get("safety", {})
SAFETY_DEFAULTS = {
    "min_relay_interval_sec": float(SCFG.get("min_relay_interval_sec", 0.5)),
    "min_command_interval_sec": float(SCFG.get("min_command_interval_sec", 0.2)),
    "flap_window_sec": float(SCFG.get("flap_window_sec", 10.0)),
    "flap_max_edge_changes": int(SCFG.get("flap_max_edge_changes", 8)),
    "flap_lockout_sec": float(SCFG.get("flap_lockout_sec", 15.0)),
}

# Availability
AVAIL_TOPIC     = f"{BASE}/cover_devices/{DEVICE_ID}/status"
PAYLOAD_ONLINE  = "online"
PAYLOAD_OFFLINE = "offline"

# Learning defaults
LDEF = CFG.get("learning", {})
LEARN_ENABLED = bool(LDEF.get("enabled", True))
LEARN_ALPHA   = float(LDEF.get("alpha", 0.2))
LEARN_CLAMP   = float(LDEF.get("clamp_percent", 0.2))

# Movement constants
TICK_SEC     = 0.1
WATCHDOG_K   = 1.5
CALIB_MARGIN = 0.99  # near-end fraction of travel time when we clamp to bounds

PERSIST = CFG.get("persistence", {})
PERSIST_PATH  = PERSIST.get("path", "./state.json")
AUTOSAVE_SEC  = int(PERSIST.get("autosave_sec", 3))

GLOBAL_START_DELAY_UP   = float(CFG.get("start_delay_up_sec", 0.4))
GLOBAL_START_DELAY_DOWN = float(CFG.get("start_delay_down_sec", 0.4))

def ensure_cover_dict(covers_cfg: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    out: Dict[str, Dict[str, Any]] = {}
    for key, meta in covers_cfg.items():
        oid = to_object_id(key)
        out[oid] = {
            "name": meta.get("name", key),
            "up_pin": int(meta["up_pin"]),
            "down_pin": int(meta["down_pin"]),
            "travel_up_cfg": float(meta.get("travel_time_up_sec")),
            "travel_down_cfg": float(meta.get("travel_time_down_sec")),
            "travel_up_learn": None,
            "travel_down_learn": None,
            "position": float(int(meta.get("initial_position", 0))),
            "target": None,
            "state": "stopped",
            "last_pub_pos": None,
            "last_tick": now(),
            "last_cmd": None,
            "cmd_start": None,
            "calibrated": bool(meta.get("calibrated", False)),
            "last_persist_bucket": None,
            "start_delay_up_sec": float(meta.get("start_delay_up_sec", GLOBAL_START_DELAY_UP)),
            "start_delay_down_sec": float(meta.get("start_delay_down_sec", GLOBAL_START_DELAY_DOWN)),
            "inhibit_until": None,
            "learn_enabled": bool(meta.get("learning", {}).get("enabled", LEARN_ENABLED)),
            "learn_alpha": float(meta.get("learning", {}).get("alpha", LEARN_ALPHA)),
            "learn_clamp": float(meta.get("learning", {}).get("clamp_percent", LEARN_CLAMP)),
            "segment_elapsed": 0.0,
            "segment_dir": None,
            "segment_start_pos": None,
            "near_end_clamped": False,
            # Safety helpers
            "guard": FlapGuard(
                min_relay_interval_sec=float(meta.get("min_relay_interval_sec", SAFETY_DEFAULTS["min_relay_interval_sec"])),
                flap_window_sec=float(meta.get("flap_window_sec", SAFETY_DEFAULTS["flap_window_sec"])),
                flap_max_edge_changes=int(meta.get("flap_max_edge_changes", SAFETY_DEFAULTS["flap_max_edge_changes"])),
                flap_lockout_sec=float(meta.get("flap_lockout_sec", SAFETY_DEFAULTS["flap_lockout_sec"])),
            ),
            "rate": RateLimiter(
                min_command_interval_sec=float(meta.get("min_command_interval_sec", SAFETY_DEFAULTS["min_command_interval_sec"]))
            ),
            "current_drive": "stop",  # 'up'|'down'|'stop'
            "lockout_until": 0.0,
        }
    return out

COVERS = ensure_cover_dict(CFG["cover"])

# Persistence
_persist_dirty = False
_last_persist = now()

def _ensure_dir(path: str):
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)

def _build_snapshot() -> Dict[str, Any]:
    snap = {}
    for oid, m in COVERS.items():
        snap[oid] = {
            "position": max(0, min(100, int(round(m["position"])))),
            "calibrated": bool(m.get("calibrated", False)),
            "travel_up_learn": m.get("travel_up_learn"),
            "travel_down_learn": m.get("travel_down_learn"),
        }
    return snap

def load_persist():
    """Load last known state from JSON persistence file."""
    global _persist_dirty, _last_persist
    try:
        if os.path.exists(PERSIST_PATH):
            with open(PERSIST_PATH, "r") as f:
                data = json.load(f)
            for oid, saved in data.items():
                if oid in COVERS:
                    m = COVERS[oid]
                    m["position"] = float(max(0, min(100, int(saved.get("position", 0)))))
                    m["calibrated"] = bool(saved.get("calibrated", False))
                    m["last_persist_bucket"] = int(m["position"]) // 5
                    if "travel_up_learn" in saved and saved["travel_up_learn"] is not None:
                        m["travel_up_learn"] = float(saved["travel_up_learn"])
                    if "travel_down_learn" in saved and saved["travel_down_learn"] is not None:
                        m["travel_down_learn"] = float(saved["travel_down_learn"])
            LOG.info(f"Persistence loaded from {PERSIST_PATH}")
        else:
            for oid, m in COVERS.items():
                m["last_persist_bucket"] = int(m["position"]) // 5
            LOG.info(f"No persistence file, starting fresh: {PERSIST_PATH}")
        _persist_dirty = False
        _last_persist = now()
    except Exception as e:
        LOG.error(f"Persistence load error: {e}")

def _mark_dirty():
    global _persist_dirty
    _persist_dirty = True

def save_persist(force: bool=False):
    """Save current state to JSON persistence file (with debounce)."""
    global _persist_dirty, _last_persist
    t = now()
    if not force and not _persist_dirty:
        return
    if not force and (t - _last_persist) < AUTOSAVE_SEC:
        return
    try:
        _ensure_dir(PERSIST_PATH)
        tmp = PERSIST_PATH + ".tmp"
        snap = _build_snapshot()
        with open(tmp, "w") as f:
            json.dump(snap, f, separators=(",", ":"), sort_keys=True)
        os.replace(tmp, PERSIST_PATH)
        _persist_dirty = False
        _last_persist = t
        LOG.debug(f"Persistence saved to {PERSIST_PATH}")
    except Exception as e:
        LOG.error(f"Persistence save error: {e}")

# GPIO init
ensure_board_mode()
for oid, m in COVERS.items():
    setup_output(m["up_pin"])
    setup_output(m["down_pin"])

# MQTT
client: mqtt.Client = make_client(AVAIL_TOPIC, PAYLOAD_OFFLINE, MQTT_USER, MQTT_PW)

def topics(oid: str) -> Dict[str, str]:
    return {
        "cmd": f"{BASE}/cover/{oid}/set",
        "set_pos": f"{BASE}/cover/{oid}/set_position",
        "pos": f"{BASE}/cover/{oid}/position",
        "state": f"{BASE}/cover/{oid}/state",
        "disc": f"{DISCOVERY}/cover/{oid}/config",
    }

def device_obj() -> Dict[str, Any]:
    return {
        "identifiers": [DEVICE_ID],
        "name": DEVICE_NAME,
        "manufacturer": DEVICE_MANU,
        "model": DEVICE_MODEL,
        "sw_version": SW_VERSION,
    }

def _persist_on_bucket_change(oid: str, pos_int: int):
    """Autosave when position crosses a 5%-bucket to limit write frequency."""
    m = COVERS[oid]
    b = pos_int // 5
    if m.get("last_persist_bucket") is None:
        m["last_persist_bucket"] = b
        return
    if b != m["last_persist_bucket"]:
        m["last_persist_bucket"] = b
        _mark_dirty()
        save_persist()

def publish_position(oid: str, force: bool=False):
    m = COVERS[oid]
    pos_int = max(0, min(100, int(round(m["position"]))))
    if not force and m["last_pub_pos"] == pos_int:
        return
    m["last_pub_pos"] = pos_int
    client.publish(topics(oid)["pos"], pos_int, retain=True)
    _mark_dirty()
    _persist_on_bucket_change(oid, pos_int)

def publish_state(oid: str, force: bool=False):
    m = COVERS[oid]
    if m["target"] is None:
        if m["position"] >= 100:
            m["state"] = "open"
        elif m["position"] <= 0:
            m["state"] = "closed"
        else:
            if m["state"] not in ("open","closed"):
                m["state"] = "stopped"
    client.publish(topics(oid)["state"], m["state"], retain=True)

def publish_discovery():
    dev = device_obj()
    for oid, m in COVERS.items():
        payload = {
            "name": m["name"],
            "unique_id": f"{DEVICE_ID}_cover_{oid}",
            "command_topic": topics(oid)["cmd"],
            "set_position_topic": topics(oid)["set_pos"],
            "position_topic": topics(oid)["pos"],
            "state_topic": topics(oid)["state"],
            "availability_topic": AVAIL_TOPIC,
            "payload_open": "OPEN",
            "payload_close": "CLOSE",
            "payload_stop": "STOP",
            "position_open": 100,
            "position_closed": 0,
            "state_open": "open",
            "state_closed": "closed",
            "state_opening": "opening",
            "state_closing": "closing",
            "state_stopped": "stopped",
            "device": dev,
        }
        client.publish(topics(oid)["disc"], json.dumps(payload), retain=True)
    for oid in COVERS:
        publish_position(oid, force=True)
        publish_state(oid, force=True)

# Motor helpers via tri-state safety
def _current_drive(oid: str) -> str:
    return COVERS[oid]["current_drive"]

def _gpio_apply_drive(oid: str, drive: str):
    m = COVERS[oid]
    if drive == "up":
        write(m["down_pin"], False)
        write(m["up_pin"], True)
    elif drive == "down":
        write(m["up_pin"], False)
        write(m["down_pin"], True)
    else:
        write(m["up_pin"], False)
        write(m["down_pin"], False)
    m["current_drive"] = drive

def _force_stop(oid: str):
    try:
        write(COVERS[oid]["up_pin"], False)
        write(COVERS[oid]["down_pin"], False)
        COVERS[oid]["current_drive"] = "stop"
    except Exception:
        pass

def _snap_and_calibrate(oid: str, pos: int):
    """Hard snap to 0/100, STOP, publish and persist."""
    m = COVERS[oid]
    m["position"] = float(max(0, min(100, int(pos))))
    m["calibrated"] = True
    m["target"] = None
    m["inhibit_until"] = None
    _gpio_apply_drive(oid, "stop")
    publish_position(oid, force=True)
    publish_state(oid, force=True)
    save_persist(force=True)
    LOG.info(f"{oid} calibrated at {int(m['position'])}%")

def _current_travel_seconds(oid: str, opening: bool) -> float:
    """Return effective full-travel time (learned if available, else configured)."""
    m = COVERS[oid]
    if opening:
        return float(m["travel_up_learn"] if m["travel_up_learn"] else m["travel_up_cfg"])
    return float(m["travel_down_learn"] if m["travel_down_learn"] else m["travel_down_cfg"])

def _apply_learning(oid: str, direction: str, effective_time: float, distance_pct: float):
    """Update learned travel time using EMA, clamped around config base."""
    m = COVERS[oid]
    if not m["learn_enabled"] or distance_pct <= 0:
        return
    dist = max(1.0, min(100.0, distance_pct))
    est_full = effective_time * (100.0 / dist)
    if direction == "up":
        base = m["travel_up_cfg"]
        prev = m["travel_up_learn"] if m["travel_up_learn"] else base
        alpha = m["learn_alpha"]
        newv = (1 - alpha) * prev + alpha * est_full
        low = base * (1.0 - m["learn_clamp"])
        high = base * (1.0 + m["learn_clamp"])
        m["travel_up_learn"] = max(low, min(high, newv))
        LOG.info(f"[LEARN] {oid} up → {m['travel_up_learn']:.2f}s (est_full={est_full:.2f}s, base={base:.2f}s)")
    else:
        base = m["travel_down_cfg"]
        prev = m["travel_down_learn"] if m["travel_down_learn"] else base
        alpha = m["learn_alpha"]
        newv = (1 - alpha) * prev + alpha * est_full
        low = base * (1.0 - m["learn_clamp"])
        high = base * (1.0 + m["learn_clamp"])
        m["travel_down_learn"] = max(low, min(high, newv))
        LOG.info(f"[LEARN] {oid} down → {m['travel_down_learn']:.2f}s (est_full={est_full:.2f}s, base={base:.2f}s)")
    _mark_dirty()
    save_persist()

def _min_runtime_required(oid: str, opening: bool) -> float:
    """Proportional minimum runtime for full OPEN/CLOSE, based on learned travel."""
    m = COVERS[oid]
    start_delay = m["start_delay_up_sec"] if opening else m["start_delay_down_sec"]
    travel = _current_travel_seconds(oid, opening)
    seg_start = m["segment_start_pos"] if m["segment_start_pos"] is not None else m["position"]
    target = 100.0 if opening else 0.0
    dist_pct = abs(seg_start - target)
    return start_delay + travel * (dist_pct / 100.0)

def start_move_to(oid: str, target: int, last_cmd: str):
    m = COVERS[oid]
    if not m["rate"].allow():
        LOG.warning(f"[SAFETY] {oid} command rate-limited")
        return
    target = max(0, min(100, int(target)))
    m["last_cmd"] = last_cmd
    m["cmd_start"] = now()
    m["near_end_clamped"] = False

    if target == int(round(m["position"])):
        m["target"] = None
        _gpio_apply_drive(oid, "stop")
        publish_state(oid)
        publish_position(oid)
        save_persist()
        return

    m["target"] = target
    m["last_tick"] = now()

    if target > m["position"]:
        m["state"] = "opening"
        res = apply_state_tristate("up", lambda: _current_drive(oid), lambda d: _gpio_apply_drive(oid, d), m["guard"], lambda: _force_stop(oid))
        m["inhibit_until"] = m["last_tick"] + max(0.0, float(m["start_delay_up_sec"]))
        m["segment_dir"] = "up"
    else:
        m["state"] = "closing"
        res = apply_state_tristate("down", lambda: _current_drive(oid), lambda d: _gpio_apply_drive(oid, d), m["guard"], lambda: _force_stop(oid))
        m["inhibit_until"] = m["last_tick"] + max(0.0, float(m["start_delay_down_sec"]))
        m["segment_dir"] = "down"

    if res.startswith("blocked"):
        m["target"] = None
        m["state"] = "stopped"
        publish_state(oid, force=True)
        publish_position(oid, force=True)
        return

    m["segment_elapsed"] = 0.0
    m["segment_start_pos"] = m["position"]
    publish_state(oid, force=True)
    publish_position(oid, force=True)
    LOG.info(f"{oid} moving to {target}% (cmd={last_cmd})")

def cmd_open(oid: str):
    start_move_to(oid, 100, "OPEN")

def cmd_close(oid: str):
    start_move_to(oid, 0, "CLOSE")

def cmd_stop(oid: str):
    m = COVERS[oid]
    m["last_cmd"] = "STOP"
    m["cmd_start"] = None
    m["target"] = None
    m["inhibit_until"] = None
    m["near_end_clamped"] = False
    apply_state_tristate("stop", lambda: _current_drive(oid), lambda d: _gpio_apply_drive(oid, d), m["guard"], lambda: _force_stop(oid))
    m["state"] = "stopped"
    publish_state(oid)
    publish_position(oid)
    save_persist()
    LOG.info(f"{oid} STOP")

# Movement tick
_last_hb = 0.0

def movement_tick():
    global _last_hb
    t = now()

    # periodic debug heartbeat
    if t - _last_hb >= 1.0:
        for _oid, _m in COVERS.items():
            LOG.debug(f"[HB] {_oid}: state={_m['state']} pos={_m['position']:.1f} target={_m['target']} cmd={_m['last_cmd']}")
        _last_hb = t

    save_persist(False)

    for oid, m in COVERS.items():
        if m["target"] is not None and m["state"] not in ("opening","closing"):
            m["state"] = "opening" if m["target"] > m["position"] else "closing"

        if m["target"] is None:
            m["last_tick"] = t
            continue

        elapsed = t - m["last_tick"]
        m["last_tick"] = t
        if elapsed <= 0:
            continue

        # skip ticks during start delay window; subtract it once elapsed
        if m["inhibit_until"] is not None:
            if t < m["inhibit_until"]:
                continue
            else:
                effective = t - m["inhibit_until"]
                elapsed = max(0.0, effective)
                m["inhibit_until"] = None

        opening = (m["state"] == "opening")
        travel = max(0.01, _current_travel_seconds(oid, opening))
        step = (elapsed / travel) * 100.0

        prev = m["position"]
        if opening:
            m["position"] = min(100.0, m["position"] + step)
        else:
            m["position"] = max(0.0, m["position"] - step)

        m["segment_elapsed"] += elapsed
        if int(prev) != int(m["position"]):
            LOG.debug(f"[TICK] {oid} {'opening' if opening else 'closing'} dt={elapsed:.3f}s step={step:.3f} pos={m['position']:.1f}")

        # watchdog: snap and STOP if runtime is far above expected
        if m["cmd_start"] and (t - m["cmd_start"]) > (WATCHDOG_K * travel):
            bound = 100 if opening else 0
            LOG.warning(f"[WATCHDOG] {oid} {'opening' if opening else 'closing'} exceeded -> snap {bound}%")
            _snap_and_calibrate(oid, bound)
            dist = abs(m["position"] - (m["segment_start_pos"] if m["segment_start_pos"] is not None else m["position"]))
            _apply_learning(oid, "up" if opening else "down", m["segment_elapsed"], max(1.0, dist))
            m["segment_elapsed"] = 0.0
            m["segment_start_pos"] = m["position"]
            m["near_end_clamped"] = False
            continue

        # reached target (OPEN side)
        if opening and m["position"] >= m["target"]:
            m["position"] = float(m["target"])
            bound_hit = (m["target"] >= 100 and m["last_cmd"] == "OPEN")
            if bound_hit:
                min_runtime = _min_runtime_required(oid, opening=True)
                if (t - (m["cmd_start"] or t)) < min_runtime:
                    publish_position(oid); publish_state(oid); save_persist()
                    LOG.debug(f"[GUARD] {oid} OPEN reached 100% early, enforcing min runtime {min_runtime:.2f}s")
                    continue
            # normal stop
            m["target"] = None
            apply_state_tristate("stop", lambda: _current_drive(oid), lambda d: _gpio_apply_drive(oid, d), m["guard"], lambda: _force_stop(oid))
            if bound_hit:
                if not m.get("near_end_clamped", False):
                    dist = abs(m["position"] - (m["segment_start_pos"] if m["segment_start_pos"] is not None else m["position"]))
                    _apply_learning(oid, "up", m["segment_elapsed"], max(1.0, dist))
                _snap_and_calibrate(oid, 100)
                m["segment_elapsed"] = 0.0
                m["segment_start_pos"] = m["position"]
                m["near_end_clamped"] = False
                continue
            publish_position(oid); publish_state(oid); save_persist()
            LOG.info(f"{oid} reached {int(m['position'])}% (OPEN side)")
            m["segment_elapsed"] = 0.0
            m["segment_start_pos"] = m["position"]
            m["near_end_clamped"] = False
            continue

        # reached target (CLOSE side)
        if (not opening) and m["position"] <= m["target"]:
            m["position"] = float(m["target"])
            bound_hit = (m["target"] <= 0 and m["last_cmd"] == "CLOSE")
            if bound_hit:
                min_runtime = _min_runtime_required(oid, opening=False)
                if (t - (m["cmd_start"] or t)) < min_runtime:
                    publish_position(oid); publish_state(oid); save_persist()
                    LOG.debug(f"[GUARD] {oid} CLOSE reached 0% early, enforcing min runtime {min_runtime:.2f}s")
                    continue
            m["target"] = None
            apply_state_tristate("stop", lambda: _current_drive(oid), lambda d: _gpio_apply_drive(oid, d), m["guard"], lambda: _force_stop(oid))
            if bound_hit:
                if not m.get("near_end_clamped", False):
                    dist = abs(m["position"] - (m["segment_start_pos"] if m["segment_start_pos"] is not None else m["position"]))
                    _apply_learning(oid, "down", m["segment_elapsed"], max(1.0, dist))
                _snap_and_calibrate(oid, 0)
                m["segment_elapsed"] = 0.0
                m["segment_start_pos"] = m["position"]
                m["near_end_clamped"] = False
                continue
            publish_position(oid); publish_state(oid); save_persist()
            LOG.info(f"{oid} reached {int(m['position'])}% (CLOSE side)")
            m["segment_elapsed"] = 0.0
            m["segment_start_pos"] = m["position"]
            m["near_end_clamped"] = False
            continue

        publish_position(oid)

        # Near-end clamp (adjusts reported position; does NOT stop the motor)
        if m["last_cmd"] in ("OPEN","CLOSE") and m["cmd_start"] is not None:
            margin = CALIB_MARGIN * travel
            if m["segment_elapsed"] >= margin and not m.get("near_end_clamped", False):
                bound = 100 if opening else 0
                m["position"] = float(bound)
                publish_position(oid)
                dist = abs(m["position"] - (m["segment_start_pos"] if m["segment_start_pos"] is not None else m["position"]))
                _apply_learning(oid, "up" if opening else "down", m["segment_elapsed"], dist)
                m["near_end_clamped"] = True
                LOG.info(f"{oid} near-end clamped to {bound}% (time guard)")

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        LOG.info("MQTT connected")
    else:
        LOG.error(f"MQTT connect failed rc={rc}")
    client.publish(AVAIL_TOPIC, PAYLOAD_ONLINE, retain=True)
    client.subscribe("homeassistant/status")
    for oid in COVERS:
        t = topics(oid)
        client.subscribe(t["cmd"])
        client.subscribe(t["set_pos"])
    publish_discovery()

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode().strip()
        topic = msg.topic
        if topic == "homeassistant/status" and payload.lower() == "online":
            LOG.info("HA online -> republish discovery")
            publish_discovery()
            return
        parts = topic.split("/")
        if len(parts) >= 4 and parts[-3] == "cover":
            oid = parts[-2]
            if oid not in COVERS:
                return
            t = topics(oid)
            m = COVERS[oid]
            # While moving, new OPEN/CLOSE should STOP (except set_position).
            if topic == t["cmd"]:
                p = payload.upper()
                if m["target"] is not None and p in ("OPEN","CLOSE"):
                    cmd_stop(oid)
                    return
                if p == "OPEN":
                    cmd_open(oid)
                elif p == "CLOSE":
                    cmd_close(oid)
                elif p == "STOP":
                    cmd_stop(oid)
                else:
                    LOG.warning(f"Unsupported cmd payload for {oid}: {payload}")
                return
            if topic == t["set_pos"]:
                try:
                    target = int(float(payload))
                except ValueError:
                    LOG.warning(f"Invalid set_position for {oid}: {payload}")
                    return
                start_move_to(oid, target, "SET")
                return
    except Exception as e:
        LOG.exception(f"on_message error: {e}")

def cleanup_and_exit(code=0):
    try:
        client.publish(AVAIL_TOPIC, PAYLOAD_OFFLINE, retain=True)
    except Exception:
        pass
    try:
        pins = []
        for m in COVERS.values():
            pins.extend([m["up_pin"], m["down_pin"]])
        safe_cleanup_low(pins)
    except Exception:
        pass
    try:
        client.loop_stop()
        client.disconnect()
        save_persist(force=True)
    except Exception:
        pass
    sys.exit(code)

def handle_sigterm(signum, frame):
    LOG.info("SIGTERM received, shutting down")
    cleanup_and_exit(0)

signal.signal(signal.SIGTERM, handle_sigterm)
signal.signal(signal.SIGINT, handle_sigterm)

def main():
    load_persist()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT, 30)
    LOG.info("shutter server started (HA discovery, availability, learning, safety)")
    client.loop_start()
    try:
        while True:
            movement_tick()
            time.sleep(TICK_SEC)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_and_exit(0)

if __name__ == "__main__":
    main()
