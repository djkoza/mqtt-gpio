# mqtt-gpio

A modular **MQTT + GPIO** stack for Raspberry Pi:

- **Shutter Server** — HA MQTT Discovery, anti-flap safety, asymmetric travel times, start-delay inhibit, watchdog, end-stop auto-calibration, travel-time learning, 5% position buckets, persistence.
- **Light Server** — HA MQTT Discovery, anti-flap safety, per-entity rate limiting.
- **ALSA Mixer Server** — HA MQTT Discovery (MQTT Number), two-way ALSA mixer sync (amixer set + alsactl monitor/poll), per-control command rate limiting.
- **Button Client** — reads buttons from **GPIO** or **ExpanderPi** (MCP23017), recognizes single/double click and long-press start/end, and publishes actions to MQTT via a queued, TTL-protected dispatcher.

---

## Features

- **Home Assistant integration** via MQTT Discovery (retained config + state).
- **Safety/anti-flap**: minimum relay spacing, per-entity command rate limits, sliding window edge counting with **lockout**.
- **Reliability**: watchdog timeouts, deduplication, self-healing state, graceful shutdown.
- **Shutters**: different up/down travel times, start-delay inhibit to compensate motor lag, bound-snap calibration, optional **learning** of travel time.
- **Persistence**: atomic JSON store for positions, calibration, and learned times.
- **Buttons**: queue with max length and TTL, reconnect backoff, GPIO or ExpanderPi inputs.
- **Ops-ready**: single YAML config, **systemd** unit files, log rotation, easy to automate with Ansible.

---

## Repository Structure

```text
mqtt-gpio/
├── apps/
│   ├── shutter_server.py
│   ├── light_server.py
│   ├── alsa_server.py
│   └── button_client.py
├── mqtt_gpio/                 # shared modules
│   ├── __init__.py
│   ├── config.py
│   ├── logging_setup.py
│   ├── mqtt_base.py
│   ├── safety.py
│   ├── gpio_driver.py
│   ├── persistence.py
│   ├── ha_discovery.py
│   └── expander.py            # ExpanderPi wrapper (optional)
├── ExpanderPi.py              # vendor library (if not installed via pip)
├── config.yaml                # single config for all apps
├── systemd/
│   ├── shutter-server.service
│   ├── light-server.service
│   ├── alsa-server.service
│   └── button-client.service
├── requirements.txt
├── LICENSE
└── README.md
```

> If you prefer installing ExpanderPi from your distro/pip, you can remove the bundled `ExpanderPi.py` and rely on the package instead.

---

## Requirements

- Raspberry Pi OS (or compatible)
- Python 3.9+
- MQTT broker (e.g., Mosquitto)
- Optional: **ABE Expander Pi** (MCP23017) for additional inputs

`requirements.txt`:
```bash
paho-mqtt>=1.6,<2
PyYAML>=6.0
RPi.GPIO>=0.7
smbus2>=0.4 # only if you use ExpanderPi
spidev>=3.5 # only if you use ExpanderPi ADC/DAC
```

--

## Install:
```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

--

## Configuration

All services read config.yaml. One file, three apps. Example (excerpt):

```bash
mqtt:
  host: 192.168.1.10
  port: 1883
  username: mqtt
  password: secret
  base_topic: home
  discovery_prefix: homeassistant

device:
  id: "rpi-gpio-node-1"
  name: "RPi GPIO Node 1"
  manufacturer: DIY
  model: "MQTT-GPIO"
  sw_version: "1.0"

logging:
  path: ./server.log
  level: INFO
  max_bytes: 1M
  backup_count: 4

persistence:
  path: ./state.json
  autosave_sec: 3

# Global safety defaults (can be overridden per entity)
safety:
  min_relay_interval_sec: 0.5     # minimal spacing between relay edges
  min_command_interval_sec: 0.2   # minimal spacing between MQTT commands per entity
  flap_window_sec: 10              # sliding window used for edge counting
  flap_max_edge_changes: 8         # edges in window to trigger lockout
  flap_lockout_sec: 15             # lockout duration after trip

# Shutters
cover:
  livingroom:
    name: "Livingroom Shutter"
    up_pin: 36
    down_pin: 37
    travel_time_up_sec: 22.0
    travel_time_down_sec: 21.0
    initial_position: 100
    # Optional cover overrides:
    start_delay_up_sec: 0.4
    start_delay_down_sec: 0.3
    learning:
      enabled: true
      alpha: 0.2
      clamp_percent: 0.2
    # Optional per-entity safety override:
    # min_relay_interval_sec: 0.6

# Lights
light:
  kitchen:
    name: "Kitchen Light"
    pin: 11
  desk:
    name: "Desk Light"
    pin: 13

# ALSA mixer controls (exposed as HA MQTT Number)
# Requires: alsa-utils (amixer, alsactl)
alsa:
  poll_sec: 1.0
  enable_monitor: true
  controls:
    dspvolume:
      name: "DSPVolume"
      card: 0
      control: "DSPVolume"
      min: 0
      max: 100
      step: 1
      unit: "%"
      icon: "mdi:volume-high"

# Button client
button_queue:
  max_limit: 200
  max_age_sec: 30

state:  # Button definitions (GPIO or expander)
  - source: "gpio"
    pin: 29
    singleClickAction:
      base_topic: "home"
      component: "cover"
      object_id: "livingroom"
      type_topic: "set"          # "set" or "set_position"
      payload: "OPEN"
    doubleClickAction:
      base_topic: "home"
      component: "cover"
      object_id: "livingroom"
      type_topic: "set_position"
      payload: "80"
    longPushStartAction:
      base_topic: "home"
      component: "cover"
      object_id: "livingroom"
      type_topic: "set"
      payload: "OPEN"
    longPushEndAction:
      base_topic: "home"
      component: "cover"
      object_id: "livingroom"
      type_topic: "set"
      payload: "STOP"

  - source: "expander"
    pin: 1
    singleClickAction:
      base_topic: "home"
      component: "light"
      object_id: "kitchen"
      type_topic: "set"
      payload: "TOGGLE"
```

--

## Home Assistant

Discovery topics are published under ${discovery_prefix}/<component>/<object_id>/config.
Runtime topics (examples):

    Shutters
        Command: home/cover/<id>/set — payload: OPEN|CLOSE|STOP
        Set position: home/cover/<id>/set_position — payload: 0..100
        State: home/cover/<id>/state — payload: open|closed|opening|closing|stopped
        Position: home/cover/<id>/position — payload: 0..100 (int)

    Lights
        Command: home/light/<id>/set — payload: ON|OFF|TOGGLE
        State: home/light/<id>/state — payload: ON|OFF

    ALSA mixer (MQTT Number)
        Command: home/alsa/<id>/set — payload: 0..100
        State: home/alsa/<id>/state — payload: 0..100 (int)

Availability (LWT): home/cover_devices/<device_id>/status and home/light_devices/<device_id>/status (online|offline).


--

## Running with systemd

Copy unit files from systemd/ and enable the services:

```
sudo cp systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable shutter-server.service light-server.service alsa-server.service button-client.service
sudo systemctl start  shutter-server.service light-server.service alsa-server.service button-client.service
```

Check status/logs:

```
systemctl status shutter-server
journalctl -u shutter-server -f
```

--

## Safety notes

    The software implements guard rails (anti-flap, minimum intervals, lockout).
    You are still responsible for wiring safety (flyback diodes, proper PSU, relay ratings).
    Use systemd restart policies and hardware fusing appropriate to your load.

--

## Development

Create a venv and install deps:

```
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

Run individual apps:

```
python apps/shutter_server.py
python apps/light_server.py
python apps/button_client.py
```
Linting (optional):

```
pip install ruff
ruff check .
```

--

## Troubleshooting

- Relays toggle too fast → tune safety.* thresholds; check for duplicate publishers.
- Shutter never reaches end → verify travel_time_up_sec/down_sec, increase start_delay_*, ensure wiring polarity matches pins.
- ExpanderPi not found → install smbus2 and spidev; ensure I²C/SPI enabled in raspi-config.
- HA entities missing → check broker creds, discovery_prefix and retained discovery messages; look at homeassistant/status.
