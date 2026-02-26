# AQUABOTS — Team 26061

**University of Arizona ENGR 498B — Spring 2026**

Autonomous surface vehicle (ASV) swarm platform.  Three boats operate as a
coordinated swarm using ESP-NOW for inter-node communication.  The leader boat
carries a Jetson Orin Nano, Starlink Mini, and IMX219 camera; follower boats
run a Raspberry Pi + ESP32-S3.

---

## Repository layout

```
AQUABOTS/
├── firmware/          ESP32-S3 swarm logic (ESP-IDF v5.x, FreeRTOS)
│   └── main/          All C source files — see firmware/main/README.md
├── ground_station/    Jetson Orin Nano Python dashboard
│   └── README.md      Setup, config, and data-pipeline docs
├── docs/              Project documents (CDR, ISR, integration plan)
│   └── pics/          Hardware and testing photos
├── reference/         Previous team's CAD files and reference code
└── README.md          This file
```

---

## Hardware overview

| Component | Leader boat | Follower boats |
|---|---|---|
| Compute | Jetson Orin Nano | Raspberry Pi |
| Comms MCU | ESP32-S3 | ESP32-S3 |
| Connectivity | Starlink Mini | — |
| Camera | IMX219 (CSI) | — |
| Obstacle sensor | DFRobot A02YYUW ultrasonic | DFRobot A02YYUW ultrasonic |
| Battery | Ovonic 6S 6000 mAh 22.2V | — |

---

## Subsystems

### `firmware/` — ESP32-S3 swarm firmware

Written in C with ESP-IDF v5.x and FreeRTOS.  All boats run the same binary;
identity is stored in NVS flash.

Key modules:

| Module | Description |
|---|---|
| `node_config` | NVS-backed node identity (node_id 1–254) |
| `swarm_transport` | ESP-NOW send/receive, ping/ack diagnostics |
| `membership` | Peer discovery, heartbeat, active member table |
| `leader_election` | Highest-node-id election with cooldown |
| `scheduler` | TDMA slot coordination (50 ms slots) |
| `command_engine` | Command parsing, routing, and execution |
| `formation` | Line, wedge, circle, column patterns |
| `obstacle_sense` | Pluggable sensor abstraction layer |
| `a02yyuw` | DFRobot A02YYUW UART driver (9600 baud) |
| `avoidance` | State-machine collision avoidance (CLEAR/WARNING/DANGER) |

See `firmware/main/README.md` for build instructions, console commands, and
integration guide.

### `ground_station/` — Jetson Orin Nano dashboard

Python Flask app serving the operator dashboard.  Key components:

| File | Description |
|---|---|
| `app.py` | Flask server — dashboard, `/api/data`, iPerf3 test |
| `serial_bridge.py` | UART bridge: ESP32-S3 → Jetson (IF-3) |
| `mqtt_client.py` | MQTT cloud telemetry via Starlink (IF-4/IF-5) |
| `config.py` | All IPs, ports, and serial settings (env-var overrides) |
| `signaling_server.py` | WebRTC signaling server (WebSocket) |

See `ground_station/README.md` for deployment instructions and environment
variable reference.

---

## Quick start

### Firmware

```bash
cd firmware
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Provision node identity at the serial console:
```
config set_id <1-254>
```

### Ground station

```bash
cd ground_station
pip install -r requirements.txt
export AQUABOT_SIGNALING_CLIENT_HOST=<jetson-lan-ip>
python app.py           # starts Flask + serial bridge + MQTT
python signaling_server.py   # separate terminal
```

Open `http://<jetson-ip>:5000` in a browser.

---

## Team

| Name | Role |
|---|---|
| John | ESP32-S3 swarm firmware |
| Mayra | Ground station dashboard |
| Jamie | Water quality sensors (pH, salinity, temperature) |
| Robert | Propulsion / ESC hardware |
| Kama | Power systems |
| Alex | Navigation / Pixhawk integration |

---

## Interfaces (from Software Integration Plan)

| ID | Description | Status |
|---|---|---|
| IF-1 | ESP32-S3 ↔ Pixhawk (MAVLink UART) | Pending |
| IF-3 | ESP32-S3 ↔ Jetson (sensor UART) | Wired, driver ready |
| IF-4/5 | Jetson ↔ Cloud (MQTT via Starlink) | Client ready, broker TBD |
| IF-6 | ESP32-S3 ↔ ESP32-S3 (ESP-NOW) | Complete |
