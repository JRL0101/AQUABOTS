# -*- coding: utf-8 -*-
"""
serial_bridge.py — UART bridge between the ESP32-S3 swarm node and the
                   Jetson Orin Nano (Interface IF-3).

The ESP32-S3 sends newline-delimited JSON frames over UART at 115200 baud.
This module runs a background thread that reads those frames, parses them, and
stores the latest values in a thread-safe state dictionary.  Flask's /api/data
endpoint and the MQTT client both read from this shared state without touching
serial I/O directly.

Message types emitted by the ESP32-S3
--------------------------------------
Sensor data (from Jamie's pH/salinity/temperature sensors):
    {"type":"sensor","node_id":1,"ph":7.2,"salinity":35.1,"temp_c":22.5}

Swarm telemetry:
    {"type":"telemetry","node_id":1,"heading_deg":45.0,"leader_id":2,
     "member_count":3,"formation":"line"}

Obstacle / avoidance state:
    {"type":"obstacle","node_id":1,"state":"WARNING","range_mm":1200,
     "bearing_deg":0}

Usage
-----
    import serial_bridge
    serial_bridge.start()          # call once at app startup
    state = serial_bridge.get_state()   # returns a snapshot dict

The bridge auto-reconnects if the serial port drops (cable pull / USB reset).
"""

import json
import logging
import threading
import time

import serial  # pyserial

import config

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Shared state — all keys initialised to None so callers can detect "not yet
# received" vs "received 0/False".
# ---------------------------------------------------------------------------
_state = {
    # sensor data
    "ph":          None,
    "salinity":    None,
    "temp_c":      None,
    # swarm telemetry
    "node_id":     None,
    "heading_deg": None,
    "leader_id":   None,
    "member_count": None,
    "formation":   None,
    # obstacle / avoidance
    "obstacle_state":   None,   # "DISABLED" | "CLEAR" | "WARNING" | "DANGER"
    "obstacle_range_mm": None,
    "obstacle_bearing_deg": None,
    # bookkeeping
    "last_rx_ts": None,         # epoch float of most recent valid frame
    "serial_ok":  False,        # True while port is open and reading
}

_lock = threading.Lock()
_thread = None


def get_state() -> dict:
    """Return a thread-safe snapshot of the latest telemetry state."""
    with _lock:
        return dict(_state)


def _update(key_values: dict) -> None:
    with _lock:
        _state.update(key_values)
        _state["last_rx_ts"] = time.time()


def _parse_frame(line: str) -> None:
    """Parse one newline-terminated JSON frame and update shared state."""
    line = line.strip()
    if not line:
        return
    try:
        msg = json.loads(line)
    except json.JSONDecodeError:
        logger.debug("Serial: non-JSON line: %s", line[:80])
        return

    msg_type = msg.get("type")

    if msg_type == "sensor":
        _update({
            "ph":       msg.get("ph"),
            "salinity": msg.get("salinity"),
            "temp_c":   msg.get("temp_c"),
            "node_id":  msg.get("node_id"),
        })

    elif msg_type == "telemetry":
        _update({
            "node_id":      msg.get("node_id"),
            "heading_deg":  msg.get("heading_deg"),
            "leader_id":    msg.get("leader_id"),
            "member_count": msg.get("member_count"),
            "formation":    msg.get("formation"),
        })

    elif msg_type == "obstacle":
        _update({
            "node_id":              msg.get("node_id"),
            "obstacle_state":       msg.get("state"),
            "obstacle_range_mm":    msg.get("range_mm"),
            "obstacle_bearing_deg": msg.get("bearing_deg"),
        })

    else:
        logger.debug("Serial: unknown message type: %s", msg_type)


def _reader_loop() -> None:
    """
    Background thread: open the serial port, read lines, parse frames.
    Reconnects automatically on error using the interval from config.
    """
    while True:
        port = config.SERIAL_PORT
        baud = config.SERIAL_BAUD
        logger.info("Serial bridge: connecting to %s @ %d baud", port, baud)

        try:
            with serial.Serial(port, baud, timeout=1.0) as ser:
                with _lock:
                    _state["serial_ok"] = True
                logger.info("Serial bridge: connected to %s", port)

                while True:
                    try:
                        raw = ser.readline()
                    except serial.SerialException as exc:
                        logger.warning("Serial read error: %s", exc)
                        break

                    if not raw:
                        continue  # timeout — keep looping

                    try:
                        line = raw.decode("utf-8", errors="replace")
                    except Exception:
                        continue

                    _parse_frame(line)

        except serial.SerialException as exc:
            logger.warning("Serial bridge: cannot open %s — %s", port, exc)
        finally:
            with _lock:
                _state["serial_ok"] = False

        logger.info("Serial bridge: reconnecting in %ds …", config.SERIAL_RECONNECT)
        time.sleep(config.SERIAL_RECONNECT)


def start() -> None:
    """
    Launch the serial reader thread.  Safe to call multiple times — only one
    thread is created.
    """
    global _thread
    if _thread is not None and _thread.is_alive():
        return
    _thread = threading.Thread(target=_reader_loop, name="serial-bridge",
                               daemon=True)
    _thread.start()
    logger.info("Serial bridge started (port=%s, baud=%d)",
                config.SERIAL_PORT, config.SERIAL_BAUD)
