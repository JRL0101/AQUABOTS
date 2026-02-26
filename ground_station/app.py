# -*- coding: utf-8 -*-
"""
app.py — Flask web server for the AquaBot ground station dashboard.

Serves the operator dashboard (HTML/CSS/JS), runs the iPerf3 network
performance test, provides a /api/data endpoint for live sensor and
telemetry data, and starts the serial bridge and MQTT client at launch.
"""

import subprocess
import json
import logging

from flask import Flask, render_template, jsonify

import config
import serial_bridge
import mqtt_client

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(name)s %(levelname)s %(message)s")

app = Flask(__name__, template_folder="templates", static_folder="static")
app.config['TEMPLATES_AUTO_RELOAD'] = True


# ---------------------------------------------------------------------------
# Home page
# ---------------------------------------------------------------------------

@app.route('/')
def index():
    # Pass the signaling URL so webrtc.js can connect without hardcoding.
    # Set AQUABOT_SIGNALING_CLIENT_HOST to the Jetson's LAN IP before launch.
    return render_template('index.html',
                           signaling_url=config.SIGNALING_URL)


# ---------------------------------------------------------------------------
# Live telemetry API  (polled by telemetry.js every 2 s)
# ---------------------------------------------------------------------------

@app.route('/api/data')
def api_data():
    """
    Return the latest sensor, telemetry, and obstacle data as JSON.
    All values are None / null if the serial bridge has not yet received them.
    """
    return jsonify(serial_bridge.get_state())


# ---------------------------------------------------------------------------
# iPerf3 network performance test
# ---------------------------------------------------------------------------

@app.route('/run_iperf')
def run_iperf():
    try:
        result = subprocess.run(
            ["iperf3", "-c", config.IPERF_HOST, "-p", str(config.IPERF_PORT), "--json"],
            capture_output=True,
            text=True,
            timeout=20,
        )

        if not result.stdout.strip():
            raise ValueError("No output from iPerf3")

        data = json.loads(result.stdout)
        end  = data.get("end", {})

        stats = {
            "upload":   round(end.get("sum_sent",     {}).get("bits_per_second", 0) / 1e6, 2),
            "download": round(end.get("sum_received", {}).get("bits_per_second", 0) / 1e6, 2),
            "latency":  round(end.get("streams", [{}])[0].get("sender", {}).get("mean_rtt", 0) / 1000, 2),
            "jitter":   round(end.get("sum", {}).get("jitter_ms",    0), 2),
            "loss":     round(end.get("sum", {}).get("lost_percent", 0), 2),
        }

        return jsonify(stats)

    except Exception as exc:
        return jsonify({"error": str(exc)})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Start the ESP32-S3 serial bridge (IF-3) — reads telemetry from swarm
    serial_bridge.start()

    # Start the MQTT cloud telemetry client (IF-4 / IF-5) via Starlink
    mqtt_client.start()

    app.run(host=config.FLASK_HOST, port=config.FLASK_PORT)
