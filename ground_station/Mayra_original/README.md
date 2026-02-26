# Mayra's Original Dashboard Code

This contains an exact, unmodified copy of your dashboard code.  Nothing here has been changed.

The working version of the dashboard is one level up in `ground_station/`.
That version has been extended to connect to the ESP32-S3 sensor pipeline and
the MQTT cloud uplink, but your logic, structure, and UI are untouched.

## What changed in the integrated version (ground_station/)

| File | What changed |
|---|---|
| `app.py` | Added `/api/data` endpoint; imports `config.py`, `serial_bridge.py`, `mqtt_client.py`; hardcoded IPs moved to `config.py` |
| `signaling_server.py` | Bind address/port read from `config.py` instead of hardcoded |
| `templates/index.html` | Added `id=` attributes to sensor/swarm fields so `telemetry.js` can populate them; added Swarm Status panel; loads `telemetry.js`; signaling URL injected by Flask |
| `static/js/webrtc.js` | Reads signaling server URL from `window.AQUABOT_SIGNALING_URL` (set by Flask template) instead of hardcoded `192.168.1.187` |

## New files added

| File | Purpose |
|---|---|
| `config.py` | All IPs, ports, and serial settings in one place; all overridable via environment variables |
| `serial_bridge.py` | Reads sensor/telemetry data from ESP32-S3 over UART |
| `mqtt_client.py` | Publishes telemetry to cloud via Starlink MQTT |
| `static/js/telemetry.js` | Polls `/api/data` every 2 s and updates the sensor/swarm fields in the dashboard |
| `requirements.txt` | Python dependencies |

## Files not changed at all

- `static/css/style.css`
- `static/js/bandwidth.js`
- `static/js/controls.js`
