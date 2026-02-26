# AquaBot Ground Station

Python/JavaScript web dashboard running on the **Jetson Orin Nano** aboard the
leader boat.  Provides the operator with a live video feed, real-time sensor
readings, swarm telemetry, and network performance metrics over Starlink.

---

## Directory layout

```
ground_station/
├── app.py                 Flask web server (port 5000)
├── signaling_server.py    WebRTC signaling server (WebSocket, port 8765)
├── serial_bridge.py       UART bridge — ESP32-S3 ↔ Jetson (IF-3)
├── mqtt_client.py         MQTT cloud telemetry via Starlink (IF-4 / IF-5)
├── config.py              Centralized configuration (IPs, ports, serial)
├── requirements.txt       Python dependencies
├── static/
│   ├── css/style.css
│   └── js/
│       ├── webrtc.js      WebRTC client (video receive + zoom/record cmds)
│       ├── telemetry.js   Polls /api/data every 2 s, updates dashboard
│       ├── bandwidth.js   iPerf3 network test trigger
│       └── controls.js    (reserved)
└── templates/
    └── index.html         Operator dashboard
```

---

## Quick start

### 1 — Install dependencies

```bash
cd ground_station
pip install -r requirements.txt
```

`iperf3` must also be installed on the Jetson:
```bash
sudo apt install iperf3
```

### 2 — Set environment variables

All IPs and ports are configurable without editing code.  Set the variables
that differ from the defaults below before launching.

| Variable | Default | Description |
|---|---|---|
| `AQUABOT_SIGNALING_CLIENT_HOST` | `localhost` | Jetson LAN IP seen by the browser (e.g. `192.168.1.187`) |
| `AQUABOT_SIGNALING_HOST` | `0.0.0.0` | Address the signaling server binds to |
| `AQUABOT_SIGNALING_PORT` | `8765` | WebSocket port for signaling |
| `AQUABOT_IPERF_HOST` | `192.168.1.77` | iPerf3 server IP |
| `AQUABOT_IPERF_PORT` | `5201` | iPerf3 port |
| `AQUABOT_FLASK_HOST` | `0.0.0.0` | Flask bind address |
| `AQUABOT_FLASK_PORT` | `5000` | Flask port |
| `AQUABOT_SERIAL_PORT` | `/dev/ttyUSB0` | ESP32-S3 serial device |
| `AQUABOT_SERIAL_BAUD` | `115200` | Serial baud rate |
| `AQUABOT_MQTT_BROKER` | `broker.hivemq.com` | MQTT broker hostname |
| `AQUABOT_MQTT_PORT` | `1883` | MQTT broker port |
| `AQUABOT_MQTT_USER` | *(empty)* | MQTT username (if required) |
| `AQUABOT_MQTT_PASS` | *(empty)* | MQTT password (if required) |
| `AQUABOT_NODE_ID` | `leader` | Node identifier for MQTT topics |

Example for deployment on the Jetson:
```bash
export AQUABOT_SIGNALING_CLIENT_HOST=192.168.1.187
export AQUABOT_SERIAL_PORT=/dev/ttyUSB0
```

### 3 — Launch the Flask dashboard (starts serial bridge + MQTT automatically)

```bash
python app.py
```

Open `http://<jetson-ip>:5000` in a browser.

### 4 — Launch the WebRTC signaling server (separate terminal)

```bash
python signaling_server.py
```

The IMX219 camera stream is sent to the signaling server by a GStreamer
pipeline on the Jetson; the browser receives it via WebRTC.

---

## Interface diagram

```
ESP32-S3 (swarm node)
    │  UART 115200 baud (IF-3)
    ▼
serial_bridge.py  ──────────► shared state dict (thread-safe)
                                    │                │
                              Flask /api/data    mqtt_client.py
                                    │                │
                              telemetry.js       Starlink Mini
                              (browser, 2 s)     (IF-4 / IF-5)
                                                      │
                                               cloud MQTT broker
```

---

## Data pipeline — sensor readings

1. **Jamie's sensors** (pH, salinity, temperature) connect to the ESP32-S3.
2. The ESP32-S3 sends newline-delimited JSON over UART:
   ```json
   {"type":"sensor","node_id":1,"ph":7.2,"salinity":35.1,"temp_c":22.5}
   ```
3. `serial_bridge.py` parses the frame and updates the shared state dict.
4. `app.py /api/data` serves the state as JSON.
5. `telemetry.js` polls `/api/data` every 2 seconds and updates the DOM.

The same pipeline carries swarm telemetry and obstacle/avoidance state.

---

## MQTT topic layout

| Topic | Direction | Payload |
|---|---|---|
| `aquabots/<node_id>/sensor` | publish | pH, salinity, temp_c |
| `aquabots/<node_id>/telemetry` | publish | heading, leader_id, formation |
| `aquabots/<node_id>/obstacle` | publish | avoidance state, range |
| `aquabots/swarm/status` | publish | combined heartbeat |
| `aquabots/command/+` | subscribe | operator commands from cloud |

---

## Pending integration items

- **ESP32-S3 TX path** — `serial_bridge.py` currently only reads; outbound
  command forwarding (Jetson → ESP32-S3) will be added when IF-3 wiring is
  complete.
- **MQTT TLS** — for production, set broker to an AWS IoT Core endpoint and
  configure TLS certificates via environment variables.
- **Mayra's signaling server** currently broadcasts WebRTC to all connected
  clients.  If security is needed, add session tokens.
