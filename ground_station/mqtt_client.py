# -*- coding: utf-8 -*-
"""
mqtt_client.py — MQTT cloud telemetry for the AquaBot ground station.
                 Uses the Starlink Mini uplink (IF-4 / IF-5).

Publishes the latest sensor and swarm state to a cloud MQTT broker at the rate
configured in config.MQTT_PUBLISH_HZ.  Also subscribes to the command topic so
that the remote operator can send commands back to the swarm.

Topic layout
------------
    aquabots/<node_id>/sensor     — pH, salinity, temperature
    aquabots/<node_id>/telemetry  — heading, leader_id, formation, member count
    aquabots/<node_id>/obstacle   — avoidance state and range
    aquabots/swarm/status         — combined heartbeat (all of the above)
    aquabots/command/+            — incoming commands from remote operator

Incoming command payload example:
    {"cmd": "formation", "pattern": "wedge", "spacing": 2000}

Usage
-----
    import mqtt_client
    mqtt_client.start()   # call once at app startup; runs in background

The client reconnects automatically if the broker is unreachable (e.g. Starlink
not yet acquired).
"""

import json
import logging
import threading
import time

import paho.mqtt.client as mqtt

import config
import serial_bridge

logger = logging.getLogger(__name__)

_client: mqtt.Client | None = None
_thread: threading.Thread | None = None


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def _on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("MQTT: connected to %s:%d", config.MQTT_BROKER, config.MQTT_PORT)
        client.subscribe(config.MQTT_CMD_TOPIC)
        logger.info("MQTT: subscribed to %s", config.MQTT_CMD_TOPIC)
    else:
        logger.warning("MQTT: connection refused (rc=%d)", rc)


def _on_disconnect(client, userdata, rc):
    if rc != 0:
        logger.warning("MQTT: unexpected disconnect (rc=%d) — will reconnect", rc)


def _on_message(client, userdata, msg):
    """
    Handle incoming commands from the remote operator.
    Payload is JSON; the command is logged here.  Wire it into the ESP32-S3
    serial bridge (IF-3) when the Jetson → ESP32 TX path is implemented.
    """
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        logger.info("MQTT command received on %s: %s", msg.topic, payload)
        # TODO (IF-3 TX): forward command to ESP32-S3 via serial_bridge when
        # the outbound serial write path is added.
    except Exception as exc:
        logger.warning("MQTT: could not parse command payload: %s", exc)


# ---------------------------------------------------------------------------
# Publisher loop
# ---------------------------------------------------------------------------

def _publish_loop() -> None:
    """
    Publish the current state from serial_bridge to MQTT at the configured
    rate.  Runs as a daemon thread.
    """
    global _client

    interval = 1.0 / max(config.MQTT_PUBLISH_HZ, 0.1)

    while True:
        try:
            _client = mqtt.Client()
            if config.MQTT_USER:
                _client.username_pw_set(config.MQTT_USER, config.MQTT_PASS)

            _client.on_connect    = _on_connect
            _client.on_disconnect = _on_disconnect
            _client.on_message    = _on_message

            # Non-blocking connect with automatic reconnect
            _client.connect_async(config.MQTT_BROKER, config.MQTT_PORT, keepalive=60)
            _client.loop_start()

            while True:
                state = serial_bridge.get_state()

                # Only publish when the serial bridge has data
                if state.get("last_rx_ts") is None:
                    time.sleep(interval)
                    continue

                ts = int(time.time())

                # --- sensor data ---
                if any(state.get(k) is not None for k in ("ph", "salinity", "temp_c")):
                    payload = json.dumps({
                        "node_id":  state["node_id"],
                        "ph":       state["ph"],
                        "salinity": state["salinity"],
                        "temp_c":   state["temp_c"],
                        "ts":       ts,
                    })
                    _client.publish(f"{config.MQTT_TOPIC_BASE}/sensor", payload, qos=0)

                # --- swarm telemetry ---
                if state.get("heading_deg") is not None:
                    payload = json.dumps({
                        "node_id":      state["node_id"],
                        "heading_deg":  state["heading_deg"],
                        "leader_id":    state["leader_id"],
                        "member_count": state["member_count"],
                        "formation":    state["formation"],
                        "ts":           ts,
                    })
                    _client.publish(f"{config.MQTT_TOPIC_BASE}/telemetry", payload, qos=0)

                # --- obstacle state ---
                if state.get("obstacle_state") is not None:
                    payload = json.dumps({
                        "node_id":    state["node_id"],
                        "state":      state["obstacle_state"],
                        "range_mm":   state["obstacle_range_mm"],
                        "bearing_deg": state["obstacle_bearing_deg"],
                        "ts":         ts,
                    })
                    _client.publish(f"{config.MQTT_TOPIC_BASE}/obstacle", payload, qos=0)

                # --- combined swarm status heartbeat ---
                _client.publish("aquabots/swarm/status", json.dumps({
                    "node_id":       state["node_id"],
                    "serial_ok":     state["serial_ok"],
                    "leader_id":     state["leader_id"],
                    "member_count":  state["member_count"],
                    "formation":     state["formation"],
                    "obstacle_state": state["obstacle_state"],
                    "ts":            ts,
                }), qos=0)

                time.sleep(interval)

        except Exception as exc:
            logger.error("MQTT publisher error: %s — retrying in 10s", exc)
            try:
                _client.loop_stop()
            except Exception:
                pass
            time.sleep(10)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def start() -> None:
    """
    Launch the MQTT publisher thread.  Safe to call multiple times — only one
    thread is created.  Call after serial_bridge.start().
    """
    global _thread
    if _thread is not None and _thread.is_alive():
        return
    _thread = threading.Thread(target=_publish_loop, name="mqtt-client",
                               daemon=True)
    _thread.start()
    logger.info("MQTT client started (broker=%s:%d, topic base=%s)",
                config.MQTT_BROKER, config.MQTT_PORT, config.MQTT_TOPIC_BASE)
