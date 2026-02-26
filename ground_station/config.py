# -*- coding: utf-8 -*-
"""
config.py — Centralized configuration for the AquaBot ground station.

All IP addresses, ports, and tunable settings live here.  Deployment-specific
values (e.g. the Jetson's LAN IP) can be overridden at runtime by setting the
corresponding environment variables listed below without touching source code.

Usage:
    import config
    print(config.IPERF_HOST)          # "192.168.1.77" (or env override)
    print(config.SIGNALING_URL)       # full ws:// URL for browser clients
"""

import os

# ---------------------------------------------------------------------------
# Flask web server
# ---------------------------------------------------------------------------
FLASK_HOST = os.environ.get("AQUABOT_FLASK_HOST", "0.0.0.0")
FLASK_PORT = int(os.environ.get("AQUABOT_FLASK_PORT", "5000"))

# ---------------------------------------------------------------------------
# WebRTC signaling server (WebSocket)
# ---------------------------------------------------------------------------
# SIGNALING_HOST  — address the websockets server *binds* to on the Jetson
# SIGNALING_CLIENT_HOST — address the *browser* uses to connect; must be
#                         the Jetson's reachable IP or hostname on the LAN
SIGNALING_HOST        = os.environ.get("AQUABOT_SIGNALING_HOST",        "0.0.0.0")
SIGNALING_PORT        = int(os.environ.get("AQUABOT_SIGNALING_PORT",    "8765"))
SIGNALING_CLIENT_HOST = os.environ.get("AQUABOT_SIGNALING_CLIENT_HOST", "localhost")

# Convenience: full WebSocket URL passed to the browser via Flask template
SIGNALING_URL = f"ws://{SIGNALING_CLIENT_HOST}:{SIGNALING_PORT}"

# ---------------------------------------------------------------------------
# iPerf3 network performance test target
# ---------------------------------------------------------------------------
IPERF_HOST = os.environ.get("AQUABOT_IPERF_HOST", "192.168.1.77")
IPERF_PORT = int(os.environ.get("AQUABOT_IPERF_PORT", "5201"))

# ---------------------------------------------------------------------------
# UART serial bridge — ESP32-S3 swarm node ↔ Jetson (IF-3)
# ---------------------------------------------------------------------------
# On the Jetson the ESP32-S3 typically appears as /dev/ttyUSB0 or /dev/ttyACM0.
# Connect the ESP32-S3 UART TX/RX to the Jetson's GPIO UART or a USB-UART
# adapter, then set AQUABOT_SERIAL_PORT accordingly.
SERIAL_PORT      = os.environ.get("AQUABOT_SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUD      = int(os.environ.get("AQUABOT_SERIAL_BAUD", "115200"))
SERIAL_RECONNECT = int(os.environ.get("AQUABOT_SERIAL_RECONNECT_S", "5"))  # seconds

# ---------------------------------------------------------------------------
# MQTT cloud telemetry via Starlink (IF-4 / IF-5)
# ---------------------------------------------------------------------------
# Default broker is the public HiveMQ instance; swap for a private broker in
# production (e.g. AWS IoT Core, Mosquitto on the ground-station laptop).
MQTT_BROKER      = os.environ.get("AQUABOT_MQTT_BROKER",  "broker.hivemq.com")
MQTT_PORT        = int(os.environ.get("AQUABOT_MQTT_PORT", "1883"))
MQTT_USER        = os.environ.get("AQUABOT_MQTT_USER",    "")
MQTT_PASS        = os.environ.get("AQUABOT_MQTT_PASS",    "")
MQTT_NODE_ID     = os.environ.get("AQUABOT_NODE_ID",      "leader")
MQTT_TOPIC_BASE  = f"aquabots/{MQTT_NODE_ID}"
MQTT_CMD_TOPIC   = "aquabots/command/#"      # subscribe for incoming commands
MQTT_PUBLISH_HZ  = float(os.environ.get("AQUABOT_MQTT_HZ", "1.0"))  # max publish rate
