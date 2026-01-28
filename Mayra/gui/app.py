# -*- coding: utf-8 -*-
from flask import Flask, render_template, jsonify
import subprocess
import json

app = Flask(__name__, template_folder="templates", static_folder="static")
app.config['TEMPLATES_AUTO_RELOAD'] = True


# ============================================================
#                        HOME PAGE
# ============================================================
@app.route('/')
def index():
    return render_template('index.html')


# ============================================================
#               IPERF3 NETWORK PERFORMANCE TEST
# ============================================================
@app.route('/run_iperf')
def run_iperf():
    try:
        print("🚀 Running iPerf3 test...")

        result = subprocess.run(
            ["iperf3", "-c", "192.168.1.77", "-p", "5201", "--json"],
            capture_output=True,
            text=True,
            timeout=20
        )

        if not result.stdout.strip():
            raise ValueError("No output from iPerf3")

        data = json.loads(result.stdout)
        end = data.get("end", {})

        upload = round(end.get("sum_sent", {}).get("bits_per_second", 0) / 1e6, 2)
        download = round(end.get("sum_received", {}).get("bits_per_second", 0) / 1e6, 2)
        latency = round(
            end.get("streams", [{}])[0].get("sender", {}).get("mean_rtt", 0) / 1000,
            2
        )

        jitter = round(end.get("sum", {}).get("jitter_ms", 0), 2)
        loss = round(end.get("sum", {}).get("lost_percent", 0), 2)

        stats = {
            "upload": upload,
            "download": download,
            "latency": latency,
            "jitter": jitter,
            "loss": loss
        }

        print("✅ iPerf3 Results:", stats)
        return jsonify(stats)

    except Exception as e:
        print("❌ Error:", e)
        return jsonify({"error": str(e)})


# ============================================================
#                        RUN SERVER
# ============================================================
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
