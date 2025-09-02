# -*- coding: utf-8 -*-
from typing import Optional
import paho.mqtt.client as mqtt

def make_client(lwt_topic: Optional[str] = None,
                lwt_offline: str = "offline",
                username: Optional[str] = None,
                password: Optional[str] = None) -> mqtt.Client:
    """Create configured MQTT client with optional LWT."""
    client = mqtt.Client()
    if lwt_topic:
        client.will_set(lwt_topic, lwt_offline, retain=True)
    if username:
        client.username_pw_set(username, password)
    # robust reconnect backoff
    client.reconnect_delay_set(min_delay=1, max_delay=30)
    return client
