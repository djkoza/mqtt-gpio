#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import signal
import sys
import datetime
import logging
from logging.handlers import RotatingFileHandler
from threading import Thread, Event, Lock
from queue import Queue, Empty
from typing import Dict, Tuple

import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

from mqtt_gpio.config import load_config
from mqtt_gpio.logging_utils import setup_rotating_logger
from mqtt_gpio.inputs import make_input
from mqtt_gpio.mqtt_base import make_client

# --- Load config ---
CFG_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config.yaml")
CFG = load_config(CFG_PATH)

MQTT_HOST = CFG["mqtt"]["host"]
MQTT_PORT = int(CFG["mqtt"]["port"])
MQTT_USER = CFG["mqtt"].get("username")
MQTT_PW   = CFG["mqtt"].get("password")

BTNQ_CFG  = CFG.get("button_queue", {})
MAX_BUTTON_QUEUE_LIMIT   = int(BTNQ_CFG.get("max_limit", 200))
MAX_BUTTON_QUEUE_AGE_SEC = int(BTNQ_CFG.get("max_age_sec", 30))

LOG = setup_rotating_logger("button_client", CFG.get("logging", {}))

STATE = CFG["state"]

# --- Try to initialize ExpanderPi (optional) ---
EXPANDER_IO = None
try:
    from mqtt_gpio.vendor import ExpanderPi
    EXPANDER_IO = ExpanderPi.IO()
    EXPANDER_IO.set_port_direction(0, 0xFF)
    EXPANDER_IO.set_port_pullups(0, 0xFF)
    EXPANDER_IO.set_port_direction(1, 0xFF)
    EXPANDER_IO.set_port_pullups(1, 0xFF)
    LOG.info("ExpanderPi initialized")
except Exception as e:
    LOG.warning(f"ExpanderPi not available: {e}")
    EXPANDER_IO = None

# Build inputs
INPUTS = []
for v in STATE:
    v["inputState"] = False
    v["isLongPush"] = False
    v["inputDateTime"] = None
    v["inputDebounceDateTime"] = None
    v["clickCounter"] = 0
    # Create reader
    try:
        inp = make_input(v.get("source"), int(v.get("pin")), EXPANDER_IO)
        v["_reader"] = inp
        INPUTS.append(v)
        LOG.debug(f"Configured input source={v.get('source')} pin={v.get('pin')}")
    except Exception as e:
        LOG.error(f"Input config error: {e}")

# --- MQTT client with reconnect backoff ---
client: mqtt.Client = make_client(username=MQTT_USER, password=MQTT_PW)

connected_event = Event()
connected_lock = Lock()
is_connected = False

def set_connected(val: bool):
    global is_connected
    with connected_lock:
        is_connected = val
        if val:
            connected_event.set()
        else:
            connected_event.clear()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        LOG.info("MQTT connected")
        set_connected(True)
    else:
        LOG.error(f"MQTT connect failed rc={rc}")
        set_connected(False)

def on_disconnect(client, userdata, rc):
    LOG.warning(f"MQTT disconnected rc={rc}")
    set_connected(False)

client.on_connect = on_connect
client.on_disconnect = on_disconnect

client.loop_start()
try:
    client.connect(MQTT_HOST, MQTT_PORT, 30)
except Exception as e:
    LOG.error(f"Initial MQTT connect error: {e}")

# --- Action queue with TTL ---
dataQueue: "Queue[Tuple[float, dict]]" = Queue()

def enqueue_action(act: dict) -> None:
    """Enqueue an action with timestamp; if full, drop oldest."""
    if dataQueue.qsize() >= MAX_BUTTON_QUEUE_LIMIT:
        try:
            dropped = dataQueue.get_nowait()
            LOG.warning(f"[QUEUE] Dropped oldest action due to max size {MAX_BUTTON_QUEUE_LIMIT}: {dropped}")
        except Empty:
            pass
    dataQueue.put((time.time(), act))
    LOG.debug(f"[QUEUE] Enqueued action: {act}")

def safe_publish(topic: str, payload: str, qos: int = 1, timeout: float = 5.0) -> bool:
    """Publish with QoS=1 and wait for confirm. Returns True/False."""
    with connected_lock:
        if not is_connected:
            return False
    try:
        info = client.publish(topic, payload=payload, qos=qos)
        if info.rc is not mqtt.MQTT_ERR_SUCCESS:
            LOG.warning(f"[MQTT] publish rc={info.rc} for {topic}")
            return False
        # paho<=1.6 has wait_for_publish() without timeout
        t0 = time.monotonic()
        while not info.is_published():
            if time.monotonic() - t0 > timeout:
                LOG.warning(f"[MQTT] publish timeout for {topic}")
                return False
            time.sleep(0.05)
        return True
    except Exception as e:
        LOG.error(f"[MQTT] publish exception for {topic}: {e}")
        return False

def publish_action(action_dict: dict) -> None:
    """Build topic and publish action; requeue on failure."""
    base_topic = action_dict['base_topic']
    component  = action_dict['component']
    object_id  = action_dict['object_id']
    payload    = str(action_dict["payload"])
    type_topic = action_dict.get('type_topic', 'set')
    topic = f"{base_topic}/{component}/{object_id}/{type_topic}"
    if not safe_publish(topic, payload):
        LOG.warning(f"MQTT offline or publish failed, requeue: {payload} -> {topic}")
        enqueue_action(action_dict)
    else:
        LOG.info(f"[MQTT] {payload} -> {topic}")

def enqueue_if_present(v: dict, key: str) -> None:
    if (
        key in v and isinstance(v[key], dict)
        and "base_topic" in v[key] and "component" in v[key]
        and "object_id" in v[key] and "payload" in v[key]
    ):
        enqueue_action(v[key])

# --- Reader thread ---
def buttonLoop():
    """Read inputs, debounce, detect click/double/long patterns and enqueue actions."""
    while True:
        time.sleep(0.01)
        for v in INPUTS:
            nowdt = datetime.datetime.now()
            currentState = v["inputState"]

            if v["inputDebounceDateTime"] is None:
                v["inputDebounceDateTime"] = nowdt
            else:
                diffDebounce = (nowdt - v["inputDebounceDateTime"]).total_seconds() * 1000
                if diffDebounce <= 10:
                    continue
                else:
                    v["inputDebounceDateTime"] = None
                    try:
                        currentState = v["_reader"].read_pressed()
                    except Exception as e:
                        LOG.exception(f"Input read error (pin={v.get('pin')}, source={v.get('source')}): {e}")
                        continue

            if currentState:
                if not v["inputState"]:
                    v["inputDateTime"] = nowdt
                else:
                    diff = None if v["inputDateTime"] is None else (nowdt - v["inputDateTime"]).total_seconds() * 1000
                    if diff is not None and diff >= 450 and not v["isLongPush"]:
                        v["inputDateTime"] = None
                        v["isLongPush"] = True
                        enqueue_if_present(v, "longPushStartAction")
                        LOG.debug(f"[BUTTON] Long press start pin={v.get('pin')}")
            else:
                if v["isLongPush"]:
                    v["isLongPush"] = False
                    enqueue_if_present(v, "longPushEndAction")
                    LOG.debug(f"[BUTTON] Long press end pin={v.get('pin')}")

                diff = None if v["inputDateTime"] is None else (nowdt - v["inputDateTime"]).total_seconds() * 1000
                if diff is not None:
                    if v["inputState"]:
                        v['clickCounter'] += 1
                    if diff >= 300:
                        if v['clickCounter'] == 1:
                            enqueue_if_present(v, "singleClickAction")
                            LOG.debug(f"[BUTTON] Single click pin={v.get('pin')}")
                        elif v['clickCounter'] == 2:
                            enqueue_if_present(v, "doubleClickAction")
                            LOG.debug(f"[BUTTON] Double click pin={v.get('pin')}")
                        v['clickCounter'] = 0
                        v["inputDateTime"] = None

            v["inputState"] = currentState

# --- Dispatcher thread ---
def dispatcherLoop():
    """Consume queue, drop expired, publish when connected."""
    while True:
        try:
            ts, action = dataQueue.get(timeout=0.5)
        except Empty:
            continue
        if (time.time() - ts) > MAX_BUTTON_QUEUE_AGE_SEC:
            LOG.warning(f"[QUEUE] Dropped expired action (> {MAX_BUTTON_QUEUE_AGE_SEC}s): {action}")
            continue
        if not connected_event.wait(timeout=5.0):
            enqueue_action(action)
            continue
        publish_action(action)

Thread(target=buttonLoop, daemon=True).start()
Thread(target=dispatcherLoop, daemon=True).start()
LOG.info("button client started (GPIO/Expander inputs → MQTT with queue & TTL)")

def cleanup_and_exit(code=0):
    try:
        client.loop_stop()
        client.disconnect()
    except Exception:
        pass
    try:
        if GPIO.getmode() is not None:
            GPIO.cleanup()
    except Exception:
        pass
    sys.exit(code)

def handle_sigterm(signum, frame):
    LOG.info("SIGTERM received, shutting down")
    cleanup_and_exit(0)

signal.signal(signal.SIGTERM, handle_sigterm)
signal.signal(signal.SIGINT, handle_sigterm)

# keep alive
while True:
    time.sleep(1)
