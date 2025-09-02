#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import signal
import sys
from typing import Dict, Any

import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt

from mqtt_gpio.config import load_config
from mqtt_gpio.logging_utils import setup_rotating_logger
from mqtt_gpio.utils import to_object_id
from mqtt_gpio.gpio_out import ensure_board_mode, setup_output, write, safe_cleanup_low
from mqtt_gpio.mqtt_base import make_client
from mqtt_gpio.safety import FlapGuard, RateLimiter
from time import monotonic as now

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
LOG = setup_rotating_logger("light_server", CFG.get("logging", {}))

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
AVAIL_TOPIC     = f"{BASE}/light_devices/{DEVICE_ID}/status"
PAYLOAD_ONLINE  = "online"
PAYLOAD_OFFLINE = "offline"

# Lights
def ensure_light_dict(light_cfg: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    out: Dict[str, Dict[str, Any]] = {}
    for key, meta in light_cfg.items():
        oid = to_object_id(key)
        out[oid] = {
            "name": meta.get("name", key),
            "pin": int(meta["pin"]),
            # Safety
            "guard": FlapGuard(
                min_relay_interval_sec=float(meta.get("min_relay_interval_sec", SAFETY_DEFAULTS["min_relay_interval_sec"])),
                flap_window_sec=float(meta.get("flap_window_sec", SAFETY_DEFAULTS["flap_window_sec"])),
                flap_max_edge_changes=int(meta.get("flap_max_edge_changes", SAFETY_DEFAULTS["flap_max_edge_changes"])),
                flap_lockout_sec=float(meta.get("flap_lockout_sec", SAFETY_DEFAULTS["flap_lockout_sec"])),
            ),
            "rate": RateLimiter(
                min_command_interval_sec=float(meta.get("min_command_interval_sec", SAFETY_DEFAULTS["min_command_interval_sec"]))
            ),
            # Runtime
            "current_on": False,
            "last_pub_state": None,
        }
    return out

LIGHT = ensure_light_dict(CFG["light"])

# GPIO init
ensure_board_mode()
for oid, m in LIGHT.items():
    setup_output(m["pin"])

# MQTT
client: mqtt.Client = make_client(AVAIL_TOPIC, PAYLOAD_OFFLINE, MQTT_USER, MQTT_PW)

def device_obj() -> Dict[str, Any]:
    return {
        "identifiers": [DEVICE_ID],
        "name": DEVICE_NAME,
        "manufacturer": DEVICE_MANU,
        "model": DEVICE_MODEL,
        "sw_version": SW_VERSION,
    }

def _state_topic(oid: str) -> str:
    return f"{BASE}/light/{oid}/state"

def _cmd_topic(oid: str) -> str:
    return f"{BASE}/light/{oid}/set"

def _publish_state(oid: str, force: bool=False):
    pin = LIGHT[oid]["pin"]
    s = "ON" if GPIO.input(pin) == GPIO.HIGH else "OFF"
    if not force and LIGHT[oid]["last_pub_state"] == s:
        return
    LIGHT[oid]["last_pub_state"] = s
    client.publish(_state_topic(oid), s, retain=True)
    LOG.debug(f"Published state {s} -> {_state_topic(oid)}")

def _apply_light(oid: str, turn_on: bool) -> str:
    m = LIGHT[oid]
    # rate limiting
    if not m["rate"].allow():
        LOG.warning(f"[SAFETY] {oid} command rate-limited")
        return "blocked-rate"

    def _cur():
        return m["current_on"]

    def _gpio_apply(on: bool):
        write(m["pin"], on)
        m["current_on"] = on

    def _force_off():
        try:
            write(m["pin"], False)
            m["current_on"] = False
        except Exception:
            pass

    # re-use single relay safety
    from mqtt_gpio.safety import apply_state_single
    result = apply_state_single(turn_on, _cur, _gpio_apply, m["guard"], _force_off)
    if result.startswith("ok"):
        _publish_state(oid)
        LOG.info(f"Light {oid} -> {'ON' if turn_on else 'OFF'} ({result})")
    else:
        _publish_state(oid, force=True)
        LOG.warning(f"Light {oid} action blocked: {result}")
    return result

def publish_discovery():
    dev = device_obj()
    for oid, m in LIGHT.items():
        disc = f"{DISCOVERY}/light/{oid}/config"
        payload = {
            "name": m["name"],
            "unique_id": f"{DEVICE_ID}_light_{oid}",
            "command_topic": _cmd_topic(oid),
            "state_topic": _state_topic(oid),
            "payload_on": "ON",
            "payload_off": "OFF",
            "availability_topic": AVAIL_TOPIC,
            "payload_available": PAYLOAD_ONLINE,
            "payload_not_available": PAYLOAD_OFFLINE,
            "device": dev,
        }
        client.publish(disc, json.dumps(payload), retain=True)
        LOG.debug(f"Discovery published for {oid}")
    for oid in LIGHT:
        _publish_state(oid, force=True)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        LOG.info("MQTT connected")
    else:
        LOG.error(f"MQTT connect failed rc={rc}")
    client.publish(AVAIL_TOPIC, PAYLOAD_ONLINE, retain=True)
    for oid in LIGHT:
        client.subscribe(_cmd_topic(oid))
    client.subscribe("homeassistant/status")
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
        if len(parts) >= 4 and parts[-1] == "set" and parts[-3] == "light":
            oid = parts[-2]
            if oid not in LIGHT:
                return
            p = payload.upper()
            if p == "ON":
                _apply_light(oid, True)
            elif p == "OFF":
                _apply_light(oid, False)
            elif p == "TOGGLE":
                _apply_light(oid, not LIGHT[oid]["current_on"])
            else:
                LOG.warning(f"Unsupported payload for {oid}: {payload}")
    except Exception as e:
        LOG.exception(f"on_message error: {e}")

def cleanup_and_exit(code=0):
    try:
        client.publish(AVAIL_TOPIC, PAYLOAD_OFFLINE, retain=True)
    except Exception:
        pass
    try:
        pins = [m["pin"] for m in LIGHT.values()]
        safe_cleanup_low(pins)
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
    client.connect(MQTT_HOST, MQTT_PORT, 30)
    LOG.info("light server started (HA discovery, availability, safety)")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_and_exit(0)

if __name__ == "__main__":
    main()
