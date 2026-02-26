# AQUABOTS PROJECT — COMPREHENSIVE SUMMARY
## Communications & Power Systems (For Claude Code Context)

**Project:** AQUABOTS Aeropak — Autonomous Underwater/Surface Swarm Drones
**Course:** ENGR 498B Spring 2026, University of Arizona (Team 26061)
**Role:** John — Software Integration Lead (Communications + Power)
**Last Updated:** February 25, 2026

---

## TEAM STRUCTURE

- **John** — Software Integration Lead: Swarm communications (ESP32-S3 / ESP-NOW), power system design
- **Jamie** — Sensors & Data Acquisition: pH, salinity, temperature, camera (IMX219)
- **Mayra** — GUI / Ground Station & Starlink Communication
- **Robert** — Propulsion & Flight Control: Pixhawk modules, motor/servo

---

## PROJECT OVERVIEW

AQUABOTS is a swarm of autonomous surface drones for environmental monitoring. Each drone carries sensors (pH, salinity, temperature), a camera, and communicates via ESP-NOW for local swarm coordination and Starlink for cloud/ground station uplink. The system uses a leader-follower architecture where the leader drone has a Starlink uplink and relays commands/data to follower drones via ESP-NOW mesh.

---

## 1. COMMUNICATIONS SYSTEM

### 1.1 Architecture

The communication system has **6 integration interfaces across 4 subsystems**:

| Interface | Connection | Protocol | Status |
|-----------|-----------|----------|--------|
| IF-1 | ESP32 ↔ Pixhawk | UART / MAVLink | Spec Week 8 |
| IF-2 | Sensors → ESP32 | I2C / ADC | Spec Week 8 |
| IF-3 | ESP32 ↔ Jetson | UART / Serial (115200) | Spec Week 8 |
| IF-4 | Drone ↔ Cloud | Starlink / MQTT | Spec Week 8 |
| IF-5 | Cloud ↔ Ground Station | MQTT + WebSocket | Spec Week 8 |
| IF-6 | Drone ↔ Drone | ESP-NOW | ✅ DONE |

### 1.2 Swarm Communication — ESP-NOW (IF-6)

**Platform:** ESP32-S3 (Xtensa dual-core @ 240 MHz)
**Framework:** ESP-IDF v5.5.x
**Protocol:** ESP-NOW (IEEE 802.11 Action Frames, Layer 2 MAC, connectionless)
**RTOS:** FreeRTOS
**Language:** C (embedded systems)

#### 9-Step Development Roadmap — Current Status: 6/9 Complete (67%)

**STEP 1: Modular Scaffolding ✅ COMPLETE**
- 9-module architecture, 20-file modular design
- ESP-NOW transport layer (2.4 GHz, 250 kbps)
- Packed binary struct packets (64 bytes) — 10x smaller than JSON
- Verified: RTT 4-5ms, RSSI -15 to -30 dBm, 0% packet loss
- Source files: `swarm_transport.c/h`

**STEP 2: NVS Node Provisioning ✅ COMPLETE**
- Persistent node identity in NVS flash (survives power cycles)
- Single firmware binary for all nodes, runtime node_id assignment (1-254)
- Console commands: `set_node_id`, `get_node_id`, `info`, `help`
- Source files: `node_config.c/h`

**STEP 3: Membership Discovery ✅ COMPLETE**
- HELLO broadcast protocol (2-second interval)
- Automatic peer discovery (no hardcoded MACs — zero-configuration)
- Dynamic member table (50 node capacity)
- Timeout detection (10 seconds), join/leave callbacks
- Verified: Discovery time 2-4 seconds, RSSI tracking -16 to -24 dBm
- Source files: `membership.c/h`

**STEP 4: Leader Election + TDMA ✅ COMPLETE**
- Distributed leader election (highest node_id wins)
- TDMA beacon broadcasts (1-second interval), 50ms time slots per node
- Automatic schedule generation, leader failover within 2-7 seconds
- Console commands: `leader`, `schedule`
- Source files: `leader_election.c/h`

**STEP 5: Command Engine ✅ COMPLETE**
- Command types: GOTO, HOLD, STOP, RESUME
- Thread-safe execution (mutex protection)
- Duplicate command detection, real-time status tracking
- Leader-only command sending enforced
- Verified: <50ms command latency, 100% success rate
- Console commands: `goto <x> <y> [heading]`, `hold`, `stop`, `resume`, `status`
- Source files: `command_engine.c/h`

**STEP 6: Formation Control ✅ COMPLETE (Verified Feb 09, 2026)**
- Four formation patterns: LINE, CHEVRON (wedge), CIRCLE, INDIVIDUAL
- Leader sets formation type + spacing, config broadcast via ESP-NOW
- Each node calculates own position from index (extensible function pointer pattern)
- Hardware verified test results:
  - LINE 2000mm: Leader (0,0), Follower (0,-2000) → PASS
  - WEDGE 2000mm: Leader (0,0), Follower (-1000,-2000) → PASS
  - WEDGE 500mm: Leader (0,0), Follower (-250,-500) → PASS
  - Spacing change: Scaled correctly → PASS
  - Non-leader command rejection: → PASS
- Console commands: `formation <type>`, `spacing <mm>`, `fmove <x> <y>`, `fstatus`

**STEP 7: Obstacle Sensor Scaffolding — PLANNED**
- Sensor interface abstraction layer
- Mock obstacle data API for testing
- Estimated ~300 lines code

**STEP 8: Ultrasonic Distance Sensor Integration — PLANNED**
- Real sensor driver, object detection
- Originally planned as mmWave radar, may use ultrasonic
- Estimated ~400 lines code

**STEP 9: Avoidance Policy — PLANNED**
- Collision avoidance algorithms, dynamic path planning
- Detection range target: 3m minimum, response time <500ms
- Estimated ~200 lines code

**FINAL STEP (after 9): Debugging and Stress Testing**

### 1.3 Swarm Architecture Details

**Topology:** Hybrid Star-Mesh ("Gateway & Swarm")
- Main Drone (Gateway): Starlink uplink + ESP-NOW Rx/Tx + Jetson Nano + ESP32
- Follower Drones (Swarm): ESP-NOW only (relays to main), sensors, Raspberry Pi + ESP32

**Routing:** Custom flooding algorithm with TTL=3 for multi-hop relay when direct LOS blocked

**Control Strategy:** "Virtual Rigid Body"
- Swarm treats leader as origin (0,0), moves as single fixed coordinate frame
- Follower derives target using leader's heading rotation matrix + offset vector
- PID controller drives motor thrust to zero distance error
- Failsafe: If data age > 3.0s → "Station Keep" (hold GPS position)

### 1.4 Performance Metrics (Verified)

| Metric | Spec | Achieved | Status |
|--------|------|----------|--------|
| Communication Range | 30m indoor | 30m+ (designed) | ✅ PASS |
| Discovery Time | <5 seconds | 2-4 seconds | ✅ PASS |
| Command Latency | <100ms | <50ms | ✅ PASS |
| RTT Latency | — | <5ms | ✅ PASS |
| RSSI | >-70 dBm | -16 to -24 dBm | ✅ PASS |
| Packet Loss | <5% | 0% | ✅ PASS |
| Max Swarm Size | 20-50 nodes | 50 (designed, tested 2) | ✅ PASS |
| Leader Election Time | <10 seconds | 5 seconds | ✅ PASS |
| TDMA Slot Duration | 50ms | 50ms | ✅ PASS |
| Throughput | — | 50 packets/sec | ✅ PASS |

### 1.5 Bandwidth Analysis

| Link | Available | Required | Margin |
|------|-----------|----------|--------|
| Swarm (ESP-NOW) | ~1,000 kbps | 10 kbps (2 drones) | 99% free |
| Satellite (Starlink) | ~5,000 kbps | 2,560 kbps (video+telemetry) | ~50% free |

### 1.6 Starlink Communication

- Starlink Mini unit (antenna + router in one)
- Direct power from 22.2V battery rail (Starlink Mini accepts 12-48V DC input)
- Ethernet connection to Jetson Orin Nano (100BASE-T, Cat5e, RJ-45)
- Worst-case tested speeds: Uplink 10 Mbps, Downlink 55 Mbps (tested in high Wi-Fi interference neighborhood)
- CDE-1 validated: Reliable two-way communication via Starlink uplink and GUI dashboard
- Video uplink: WebRTC
- Telemetry: MQTT

### 1.7 Code Quality

- Total production code: ~2,350 lines
- Modules complete: 6/9 (67%)
- Thread-safe: Yes (mutex protection throughout)
- Memory leaks: None detected
- Documentation: 767-line comprehensive README

---

## 2. POWER SYSTEM

### 2.1 Design Evolution (PDR → CDR → ISR)

**Original PDR Design (deprecated):**
- 24V LiFePO4 battery
- 48V boost converter + PoE injector to power Starlink
- 19V buck converter
- BMS (Battery Management System)
- Complex power chain

**CDR Design (major simplification):**
- Switched to 6S LiPo battery (22.2V, 6000mAh, 133.2 Wh)
- Eliminated 48V boost converter and PoE injector entirely
- Starlink Mini powered directly from 22.2V rail (accepts 12-48V)
- Eliminated BMS (LiPo doesn't need external BMS like LiFePO4)
- Simpler, lighter, more reliable

**ISR Design (current — fuse protection upgrades):**
- Core architecture unchanged from CDR
- Main fuse upgraded: 80A → 150A (MRBF fuse holder)
- Added dedicated 125A motor fuse (MIDI fuse block)
- Proper fuse blocks: MRBF + MIDI + ST Blade ATO
- Added thermal pads for heat management
- All parts ordered and awaiting delivery

**Post-ISR Update (judge recommendation):**
- Wiring upgraded to 12 AWG (from 14 AWG) for main power runs per ISR judge recommendation

### 2.2 Current Power Architecture (ISR)

**Battery:** Ovonic 6S LiPo, 22.2V nominal, 6000mAh, 133.2 Wh, 120C (720A) discharge limit
**Main Bus:** (+) Busbar → 22.2V Rail → (-) Busbar
**Safety:** MRBF 150A main fuse → Manual kill switch

**Power Distribution:**

| Load | Fuse | Wire Gauge | Voltage | Max Current | Cruise Power |
|------|------|-----------|---------|-------------|-------------|
| ESC → Motor (Apisqueen 5080) | MIDI 125A | 12 AWG (upgraded from 8/10 AWG on ISR slides) | 22.2V direct | 54A max | 60W |
| Starlink Mini | ATO 7.5A slow-blow | 16 AWG | 22V direct | 2.7A | 70W |
| 5V Power Module → Pixhawk 4X FC | ATO 1A slow-blow | 18 AWG | 5V | — | — |
| 12V Buck → Jetson Orin Nano + Fan | ATO 7.5A slow-blow | 18 AWG | 12V | 1.3A | 15W |
| 5V Buck → ESP32 | ATO 1A slow-blow | 18 AWG | 5V | — | 5W |
| BEC (Castle) → Rudder Servo | ATO 2A slow-blow | 18 AWG | 7.4V | 1.0A | 5W |
| Sensors (GPS, IMU, Env) | via 3.3V reg | — | 3.3V | 0.3A | 5W |

**Battery → Kill Switch → Busbar trunk:** 4 AWG (Red+/Blk-)

### 2.3 Power Budget Summary

| Subsystem | Max Power | Cruise Power | Max Current |
|-----------|-----------|-------------|-------------|
| Propulsion (motor) | 1200W | 60W | 54A |
| Comms (Starlink) | 105W | 70W | 4.7A |
| Compute (Jetson + Fan) | 28W | 15W | 1.3A |
| Control (ESP32 + Servo) | 22W | 5W | 1.0A |
| Payload (Sensors) | 6W | 5W | 0.3A |
| **TOTAL** | **~1361W** | **~155W** | **~61.3A** |

**Cruise Endurance:** 133.2 Wh / 155W ≈ **51 minutes**
**Safety Margin:** Max draw 61.3A is well within 150A main fuse and 720A battery discharge limit

### 2.4 Signal & Data Interconnects

| Connection | Protocol | Signal | Cable | Connector |
|-----------|----------|--------|-------|-----------|
| ESP32 → Pixhawk (IF-1) | MAVLink / UART1 | TX/RX 3.3V | 22 AWG | JST-SH 4-pin |
| ESP32 → Jetson (IF-3) | UART2 115200 | TX/RX 3.3V | 22 AWG | JST-SH 4-pin |
| ESP32 → ESC (PWM) | PWM 50Hz | 3.3V logic | 22 AWG | Servo 3-pin |
| Jetson → Camera (CSI) | MIPI CSI-2 | LVDS diff | Ribbon | 15-pin FPC |
| Jetson → Starlink | Ethernet | 100BASE-T | Cat5e | RJ-45 |

### 2.5 Parts Procurement (All Ordered as of ISR)

| Part | Qty | Price | Status |
|------|-----|-------|--------|
| 6S LiPo Battery | 1 | $66 | ORDERED |
| MRBF Fuse Holder | 1 | $25 | ORDERED |
| MIDI Fuse Block | 1 | $38 | ORDERED |
| ST Blade Fuse Block | 1 | $47 | ORDERED |
| 150A Fuse | 1 | $18 | ORDERED |
| 125A Fuse | 1 | $12 | ORDERED |
| Bus Bars (+/-) | 2 | $100 | ORDERED |
| Manual Kill Switch | 1 | $25 | ORDERED |
| Castle BEC (7.4V) | 1 | $47 | ORDERED |
| Thermal Pads + ATO Fuses + DC Pigtails | — | $39 | ORDERED |
| **Total** | | **$417** | |

---

## 3. SOFTWARE INTEGRATION FRAMEWORK

### 3.1 Overall Status: 5/14 major items complete (35%)

### 3.2 Five-Phase Integration Roadmap

**PHASE 1: Communications Layer ✅ COMPLETE**
- 1.1 ESP-NOW Transport Layer ✅
- 1.2 Node Identity Management ✅
- 1.3 Swarm Discovery Protocol ✅
- 1.4 Leader Election & TDMA ✅
- 1.5 Command Distribution ✅

**PHASE 2: Motion Control Integration 🟡 IN PROGRESS**
- 2.1 Propulsion Interface Layer 🔲 (motor control API, PWM, thruster mapping)
- 2.2 Navigation Controller 🔲 (waypoint nav, path planning, heading control)
- 2.3 HOLD Position Controller 🔲 (station-keeping, drift compensation)

**PHASE 3: Formation Control 🔲 PLANNED**
- 3.1 Formation Algorithm 🔲 (line, wedge, circle, column implementation)
- 3.2 Formation Maintenance 🔲 (error correction, stability, dropout handling)

**PHASE 4: Obstacle Avoidance 🔲 PLANNED**
- 4.1 Sensor Interface 🔲 (sensor driver, mock data)
- 4.2 Collision Avoidance 🔲 (detection, path replanning)

**PHASE 5: System Integration & Testing 🔲 PLANNED**
- 5.1 Hardware Integration Testing 🔲
- 5.2 Performance Validation 🔲

### 3.3 Integration Schedule

| Week | Task | Status |
|------|------|--------|
| 1-2 | ESP-NOW transport layer | ✅ Complete |
| 3 | Node provisioning | ✅ Complete |
| 4-5 | Membership discovery | ✅ Complete |
| 6-7 | Leader election + TDMA | ✅ Complete |
| 8 | Command engine | ✅ Complete |
| 9-10 | Propulsion interface layer | 🟡 Current |
| 11-12 | Navigation controller | 🔲 Upcoming |
| 13-14 | Formation algorithms | 🔲 Upcoming |
| 15-16 | Sensor integration | 🔲 Upcoming |
| 17-18 | Collision avoidance | 🔲 Upcoming |
| 19-20 | Full system integration testing | 🔲 Upcoming |

### 3.4 Critical Path

Communications ✅ → Propulsion 🔴 → Navigation → Formation → Avoidance → System Test

**Current Blocker:** Propulsion hardware integration (waiting on thruster mounting & ESC setup from Robert)
**Mitigation:** Can develop navigation algorithms with mock propulsion data

---

## 4. REQUIREMENTS VERIFICATION (SRVM)

### 4.1 Communications Requirements

| Requirement | Method | Status | Evidence |
|------------|--------|--------|----------|
| 4.4.2 Inter-Drone Comms | TEST | ✅ PASS | RTT <5ms, 0% packet loss |
| 4.4.2 Inter-Drone Comms | DEMONSTRATION | ✅ PASS | Auto-discovery, synchronized commands |
| 4.4.2 Inter-Drone Comms | ANALYSIS | 🟡 In Progress | Scalability modeling (Wk 9) |
| 4.4.2 Inter-Drone Comms | INSPECTION | 🔲 Planned | Hardware/antenna check (Wk 12) |
| 5.4.2 Inter-Drone Comms | — | ✅ ESP-NOW verified | Validate 100m range at pool test |
| 5.3.1 Survivability | TEST | 🟡 Partial | Comms pass (0% loss calm), stormy test Wk 9 |
| 5.4.3 Data Relay | DEMO | 🟡 Partial | Position data OK, sensor pipeline pending Wk 11 |
| 4.4.1 Autonomy & Control | DEMO | 🟡 Partial | Commands OK, formation control done (Step 6) |
| 5.4.6 Swarm Control | — | 🟡 Partial | ESP-NOW done, need ESP32→Jetson bridge + MQTT Wk 12 |
| 4.1.2 Swarm Control | DEMO | ✅ PASS | Leader election, coordinated execution |

---

## 5. KEY DESIGN DECISIONS & CHANGES THROUGHOUT PROJECT

1. **Power simplification (PDR→CDR):** Eliminated 48V boost converter and PoE injector for Starlink. Discovered Starlink Mini accepts 12-48V DC directly, so the 22.2V LiPo can power it without conversion. Major reduction in complexity, weight, cost, and failure points.

2. **Battery change (PDR→CDR):** Switched from 24V LiFePO4 with BMS to 6S LiPo (22.2V). LiPo offers better energy density for marine application.

3. **Camera change (PDR→CDR):** Replaced GoPro Hero 7 Black with IMX219 — lower power, lower latency, CSI interface to Jetson.

4. **Fuse upgrades (CDR→ISR):** Main fuse 80A→150A (MRBF), added dedicated 125A motor fuse (MIDI), proper fuse blocks, thermal management.

5. **Wiring upgrade (post-ISR):** Upgraded main power wiring from 14 AWG to 12 AWG per ISR judge recommendation.

6. **Follower drone simplification (CDR):** Follower drones use Raspberry Pi + ESP32 (no Starlink, no Jetson). Only the main/gateway drone has Starlink uplink.

7. **Obstacle sensor decision pending:** Originally planned mmWave radar (Step 8), may use ultrasonic distance sensor instead. Decision still open.

---

## 6. DELIVERABLES TIMELINE

| Deliverable | Date | Description |
|------------|------|-------------|
| Integrated Sensor Suite | 12/2/2025 | pH, temp, salinity sensors |
| Mounted Cameras | 12/2/2025 | IMX219 with storage/streaming |
| Communication Module | 1/22/2026 | Starlink integrated with all components & GS |
| Propulsion System | 2/19/2026 | Functional propulsion w/ motor control |
| Operational Ground Station | 3/3/2026 | GUI + backend for live data |
| Functional Hull Prototype | 3/5/2026 | Hull integrated with all components |
| Manual | 4/14/2026 | Documentation and user guides |
| Project Final Report | 5/5/2026 | Summary, system specs, results |

---

## 7. DOCUMENTS PRODUCED DURING DISCUSSIONS

The following documents were created through conversations in this project and are available in the project knowledge:

1. **ISR_SWARM_COMMUNICATIONS_SUMMARY.md** — Comprehensive summary of Steps 1-5 implementation, verification results, SRVM mapping, performance metrics, and technology stack details.

2. **SOFTWARE_INTEGRATION_ACTION_ITEM.md** — Full 5-phase integration roadmap with 14 items, dependencies, schedule, risk assessment, testing infrastructure, and success metrics.

3. **ISR_READINESS_CHECKLIST.md** — Pre-ISR preparation guide covering demo preparation, SRVM mapping clarity, range testing, propulsion integration planning, slide structure recommendations, and Q&A preparation.

4. **ISR Presentation Slides** — Power subsystem and communications subsystem slide decks were discussed and refined for the ISR presentation.

---

## 8. IMPORTANT NOTES FOR CLAUDE CODE

- John prefers **technical accuracy over promotional language**. Use verified test results and concrete metrics, not "braggy" stats like lines of code.
- The SRVM (System Requirements Verification Matrix) format is critical for formal engineering reviews.
- When generating documents, John likes **Markdown and Typst** formats (Typst for professional PDF compilation).
- The project uses **GitHub** for code management.
- ESP-IDF v5.5.x is the development framework — all code is C for embedded systems.
- The ESP32-S3 firmware is a **single binary** that runs on all nodes; identity is provisioned at runtime via NVS.
- Formation control (Step 6) was completed and verified on hardware as of Feb 09, 2026, after the ISR_SWARM_COMMUNICATIONS_SUMMARY.md was written (which only covered Steps 1-5). The ISR slides reflect 6/9 steps complete.
- The next coding work is Steps 7-9 (obstacle sensing and avoidance) and the remaining integration phases (propulsion interface, navigation, etc.).
- Wiring was upgraded to 12 AWG post-ISR per judge recommendation — this is the most recent change.
