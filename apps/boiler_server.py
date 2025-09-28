#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import logging
import os
import signal
import sys
import time
from typing import Dict, Any

import yaml
import paho.mqtt.client as mqtt

try:
    import RPi.GPIO as GPIO
except Exception as e:
    raise SystemExit(f"RPi.GPIO not available: {e}")

LOG = logging.getLogger("boiler_server")

DEFAULTS = {
    "mqtt": {
        "host": "127.0.0.1",
        "port": 1883,
        "username": None,
        "password": None,
        "base_topic": "home",
        "discovery_prefix": "homeassistant",
        "retain": True,
        "qos": 0,
    },
    "device": {
        "id": "rpi-gpio-node",
        "name": "RPi GPIO Node",
        "manufacturer": "DIY",
        "model": "MQTT-GPIO",
        "sw_version": "1.0",
    },
    "logging": {
        "level": "INFO",
    },
    "safety": {
        "min_command_interval_sec": 0.2,
    },
    "boiler": {
        # example structure (object_id: {name, pin, active_high, inverted})
    },
}

class BoilerEntity:
    def __init__(self, object_id: str, cfg: Dict[str, Any], base_topic: str, discovery_prefix: str, device: Dict[str, Any]):
        self.object_id = object_id
        self.name = cfg.get("name", f"Boiler {object_id}")

        self.pin = int(cfg["pin"])
        self.active_high = bool(cfg.get("active_high", True))
        self.initial_state = cfg.get("initial_state", "OFF").upper()
        self.base_topic = base_topic.rstrip("/")
        self.discovery_prefix = discovery_prefix.rstrip("/")
        self.device = device
        self.state_topic = f"{self.base_topic}/boiler/{self.object_id}/state"
        self.command_topic = f"{self.base_topic}/boiler/{self.object_id}/set"
        self.availability_topic = f"{self.base_topic}/boiler/{self.object_id}/availability"
        self.current_state = None  # "ON"/"OFF"

        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW if self.active_high else GPIO.HIGH)

    def _write_gpio(self, on: bool):
        if self.active_high:
            GPIO.output(self.pin, GPIO.HIGH if on else GPIO.LOW)
        else:
            GPIO.output(self.pin, GPIO.LOW if on else GPIO.HIGH)

    def apply_state(self, new_state: str):
        new_state = "ON" if new_state.upper() in ("ON", "1", "TRUE") else "OFF"
        if new_state == self.current_state:
            return False
        self._write_gpio(new_state == "ON")
        self.current_state = new_state
        return True

    def readback_state(self) -> str:
        val = GPIO.input(self.pin)

        logical_on = (val == GPIO.HIGH) if self.active_high else (val == GPIO.LOW)
        return "ON" if logical_on else "OFF"

    def discovery_payload(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "unique_id": f"{self.device.get('id','rpi')}_boiler_{self.object_id}",
            "command_topic": self.command_topic,
            "state_topic": self.state_topic,
            "availability_topic": self.availability_topic,
            "payload_on": "ON",
            "payload_off": "OFF",
            "icon": "mdi:water-boiler",
            "device": {
                "identifiers": [self.device.get("id", "rpi")],
                "manufacturer": self.device.get("manufacturer", "DIY"),
                "model": self.device.get("model", "MQTT-GPIO"),
                "name": self.device.get("name", "RPi GPIO Node"),
                "sw_version": self.device.get("sw_version", "1.0"),
            },
        }

    def discovery_topic(self) -> str:
        return f"{self.discovery_prefix}/switch/{self.device.get('id','rpi')}/{self.object_id}/config"


class BoilerServer:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.mqtt_cfg = {**DEFAULTS["mqtt"], **cfg.get("mqtt", {})}
        self.device = {**DEFAULTS["device"], **cfg.get("device", {})}
        self.safety = {**DEFAULTS["safety"], **cfg.get("safety", {})}
        self.entities: Dict[str, BoilerEntity] = {}
        self.client = mqtt.Client(client_id=f"boiler-{self.device['id']}", clean_session=True)
        self.last_cmd_ts: Dict[str, float] = {}
        self.retain = bool(self.mqtt_cfg.get("retain", True))
        self.qos = int(self.mqtt_cfg.get("qos", 0))

        if self.mqtt_cfg.get("username"):
            self.client.username_pw_set(self.mqtt_cfg["username"], self.mqtt_cfg.get("password"))

        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

        # GPIO init
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        # build entities
        boiler_section = cfg.get("boiler", {})
        if not isinstance(boiler_section, dict) or not boiler_section:
            raise SystemExit("Config 'boiler' section is empty.")
        for object_id, ent_cfg in boiler_section.items():
            self.entities[object_id] = BoilerEntity(
                object_id=object_id,
                cfg=ent_cfg,
                base_topic=self.mqtt_cfg["base_topic"],
                discovery_prefix=self.mqtt_cfg["discovery_prefix"],
                device=self.device,
            )

    # MQTT callbacks
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            LOG.info("Connected to MQTT.")
            # availability online + discovery + subscriptions
            for oid, ent in self.entities.items():
                client.publish(ent.availability_topic, "online", qos=self.qos, retain=True)
                client.publish(ent.discovery_topic(), json.dumps(ent.discovery_payload()), qos=self.qos, retain=True)
                client.subscribe(ent.command_topic, qos=self.qos)
                # initial state
                init = ent.initial_state
                if init.upper() not in ("ON", "OFF"):
                    init = ent.readback_state()
                ent.apply_state(init)
                client.publish(ent.state_topic, ent.readback_state(), qos=self.qos, retain=self.retain)
        else:
            LOG.error("MQTT connect failed rc=%s", rc)

    def _on_disconnect(self, client, userdata, rc):
        LOG.warning("Disconnected from MQTT rc=%s", rc)

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode().strip().upper()
        except Exception:
            payload = ""
        # find entity by topic
        for oid, ent in self.entities.items():
            if msg.topic == ent.command_topic:
                now = time.time()
                min_gap = float(self.safety.get("min_command_interval_sec", 0.2))
                last = self.last_cmd_ts.get(oid, 0.0)
                if (now - last) < min_gap:
                    LOG.warning("Rate-limited command for %s (gap %.3fs < %.3fs)", oid, (now-last), min_gap)
                    return
                self.last_cmd_ts[oid] = now
                new_state = "ON" if payload in ("ON", "1", "TRUE") else "OFF"
                changed = ent.apply_state(new_state)
                # Publish state regardless to keep HA in sync
                client.publish(ent.state_topic, ent.readback_state(), qos=self.qos, retain=self.retain)
                LOG.info("Set %s → %s (changed=%s)", oid, new_state, changed)
                return

    def start(self):
        # LWT (offline)
        self.client.will_set(
            f"{self.mqtt_cfg['base_topic']}/boiler/{self.device['id']}/availability",
            payload="offline",
            qos=self.qos,
            retain=True,
        )
        self.client.connect(self.mqtt_cfg["host"], int(self.mqtt_cfg["port"]))
        self.client.loop_start()

    def stop(self):
        try:
            for ent in self.entities.values():
                self.client.publish(ent.availability_topic, "offline", qos=self.qos, retain=True)
        except Exception:
            pass
        try:
            self.client.loop_stop()
            self.client.disconnect()
        finally:
            GPIO.cleanup()

def load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cfg = {}
    for k, v in DEFAULTS.items():
        if isinstance(v, dict):
            cfg[k] = {**v, **data.get(k, {})}
        else:
            cfg[k] = data.get(k, v)
    if "boiler" in data:
        cfg["boiler"] = data["boiler"]
    return cfg

def main():
    ap = argparse.ArgumentParser(description="MQTT Boiler Server")
    ap.add_argument("-c", "--config", default=os.environ.get("MQTT_GPIO_CONFIG", "config.yaml"), help="path to config.yaml")
    args = ap.parse_args()

    # logging
    level = os.environ.get("BOILER_LOGLEVEL", None)
    cfg = load_config(args.config)
    if not level:
        level = cfg.get("logging", {}).get("level", "INFO")
    logging.basicConfig(
        level=getattr(logging, str(level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    server = BoilerServer(cfg)

    def _sigterm(signum, frame):
        LOG.info("Shutting down...")
        server.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigterm)
    signal.signal(signal.SIGTERM, _sigterm)

    server.start()
    LOG.info("MQTT Boiler Server started.")
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
