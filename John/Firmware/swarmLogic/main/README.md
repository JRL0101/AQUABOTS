# AQUABOTS — ESP32-S3 Swarm Framework

**Platform:** ESP32-S3 with ESP-IDF v5.5.x
**Protocol:** ESP-NOW (2.4 GHz, channel 1)
**Target:** 20–50 node autonomous surface vehicle swarm

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Module Reference](#module-reference)
4. [Hardware Setup](#hardware-setup)
5. [Building and Flashing](#building-and-flashing)
6. [Node Provisioning](#node-provisioning)
7. [Console Command Reference](#console-command-reference)
8. [Integration Guide](#integration-guide)
9. [Testing and Verification](#testing-and-verification)
10. [Troubleshooting](#troubleshooting)
11. [Performance Reference](#performance-reference)

---

## Overview

AQUABOTS is a distributed swarm coordination framework for ESP32-S3 microcontrollers
targeting autonomous surface vehicles (ASVs). All coordination is done peer-to-peer
over ESP-NOW — no router, broker, or infrastructure is required.

### Capabilities

- **Zero-configuration networking** — nodes discover each other automatically via broadcast
- **Persistent identity** — each node has a unique ID stored in flash that survives power cycles
- **Distributed leader election** — highest node_id automatically becomes the swarm leader
- **Time-slotted communication** — TDMA scheduling prevents radio collisions at scale
- **Synchronized command execution** — the leader sends GOTO/HOLD/STOP/RESUME to the entire swarm
- **Formation control** — four built-in patterns (line, wedge, circle, column) with configurable spacing
- **Obstacle sensing** — pluggable sensor driver system with swarm-wide scan fusion
- **Collision avoidance** — autonomous safety zones with automatic stop and alternate-path suggestion
- **Resilient design** — automatic leader failover if the current leader goes offline

### Technology Stack

| Component | Technology |
|-----------|------------|
| Hardware | ESP32-S3 (Xtensa dual-core @ 240 MHz) |
| Framework | ESP-IDF v5.5.x |
| RTOS | FreeRTOS |
| Radio | ESP-NOW (2.4 GHz) |
| Storage | NVS (Non-Volatile Storage) |
| Console | USB Serial/JTAG |

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                       APPLICATION LAYER                       │
├────────────────────┬─────────────────────┬───────────────────┤
│  Formation Control │   Command Engine    │ Collision Avoid   │
├────────────────────┴──────┬──────────────┴───────────────────┤
│     Leader Election       │    Obstacle Sensing + Fusion      │
├───────────────────────────┴───────────────────────────────────┤
│                  Membership Discovery                          │
├───────────────────────────────────────────────────────────────┤
│           Node Configuration (NVS identity)                   │
├───────────────────────────────────────────────────────────────┤
│              Swarm Transport (ESP-NOW)                         │
└───────────────────────────────────────────────────────────────┘
```

### Data Flow

```
Physical sensor (A02YYUW)
        │  UART 9600 baud
        ▼
   a02yyuw driver ──► obstacle_sense (local scan, 10 Hz)
                              │
                              ├──► broadcast to swarm (2 Hz, MSG_TYPE_OBSTACLE)
                              │
                              ├──► receives remote scans from peers
                              │
                              └──► avoidance (polls every 250 ms)
                                        │
                                  DANGER zone ──► CMD_STOP (leader only)
                                  CLEAR found ──► suggest alternate bearing
```

---

## Module Reference

### swarm_transport (`swarm_transport.h / .c`)

ESP-NOW radio abstraction layer. Manages peer registration, send/receive dispatch,
and supports multiple simultaneous receive callbacks via a callback table.

- **Channel:** 1 (configurable via `TRANSPORT_CHANNEL`)
- **Max peers:** 20 (configurable via `TRANSPORT_MAX_PEERS`)
- **Receive dispatch:** up to 8 callbacks registered simultaneously

---

### node_config (`node_config.h / .c`)

Persistent node identity backed by NVS flash. Provides a serial provisioning
console so a single firmware binary can be flashed to every node; each board
is then assigned its unique node_id via `set_node_id`.

- **Valid node_id range:** 1–254
- **Storage:** NVS namespace `node_cfg`, key `node_id`
- **Console commands:** `set_node_id`, `get_node_id`, `info`, `help`

---

### membership (`membership.h / .c`)

Zero-configuration peer discovery. Each provisioned node broadcasts a HELLO
packet every 2 seconds. Nodes that stop broadcasting are removed from the
member table after a 10-second timeout.

- **HELLO interval:** 2 000 ms
- **Peer timeout:** 10 000 ms
- **Max peers:** 50 (configurable via `SWARM_MAX_NODES` in `swarm_protocol.h`)
- **Message:** `MSG_TYPE_HELLO` (0x02)

---

### leader_election (`leader_election.h / .c`)

Distributed leader election by highest node_id. The elected leader generates
a TDMA slot schedule and broadcasts it in beacon packets every second. All
nodes receive the beacon and update their slot timing accordingly.

- **Election rule:** highest node_id among all known members
- **Election trigger:** every 5 seconds, or when membership changes
- **TDMA slot duration:** 50 ms per node
- **Beacon interval:** 1 000 ms
- **Message:** `MSG_TYPE_BEACON` (0x03)

**Example schedule (2 nodes, IDs 1 and 2):**
```
Slot 0 (  0– 50 ms): Node 2 (leader)
Slot 1 ( 50–100 ms): Node 1
[cycle repeats every 100 ms]
```

---

### command_engine (`command_engine.h / .c`)

Swarm command distribution. Only the leader can originate commands; followers
receive and execute them. All nodes track execution state independently.
Duplicate detection (via 32-bit command ID) prevents re-execution of
retransmitted packets.

- **Commands:** `GOTO`, `HOLD`, `STOP`, `RESUME`
- **Message:** `MSG_TYPE_COMMAND` (0x04)
- **Thread-safe:** mutex-protected state machine

| Command | Parameters | Behaviour |
|---------|------------|-----------|
| GOTO | x, y, heading | Navigate swarm to waypoint |
| HOLD | — | Maintain current position |
| STOP | — | Emergency stop (motors off) |
| RESUME | — | Resume last active command |

---

### formation (`formation.h / .c`)

Coordinated spatial formations. The leader calculates relative position offsets
for each node and broadcasts the formation plan. Each node receives its assigned
offset and applies it to the shared formation centre to obtain its world position.

New patterns can be added without modifying existing code by calling
`formation_register_pattern()` with a custom calculator function.

- **Built-in patterns:** line, wedge, circle, column
- **Default spacing:** 2 000 mm (configurable 500–10 000 mm)
- **Message:** `MSG_TYPE_FORMATION` (0x05)

| Pattern | Description |
|---------|-------------|
| `line` | Single file, leader at front |
| `wedge` | V-shape with leader at the point |
| `circle` | Nodes evenly spaced on a ring |
| `column` | Side-by-side parallel columns |

---

### obstacle_sense (`obstacle_sense.h / .c`)

Sensor abstraction with swarm-wide scan fusion. A pluggable `sensor_driver_fn`
is called at 10 Hz to collect local detections. The local scan is broadcast at
2 Hz to all peers. Remote scans from other nodes are stored and automatically
expired after 2 seconds if not refreshed.

All query APIs (`obstacle_sense_get_closest`, `obstacle_sense_path_clear`,
`obstacle_sense_get_all_scans`) operate across both local and all current remote
scans, giving every node a unified obstacle picture of the entire swarm's field
of view.

**Mock mode** is active by default. It allows full swarm testing without any
physical sensors attached. Use `mock_add` to inject test obstacles from the
console.

- **Local scan rate:** 10 Hz (100 ms interval)
- **Broadcast rate:** 2 Hz (500 ms interval)
- **Scan timeout:** 2 000 ms
- **Max tracked nodes:** 16
- **Max detections per scan:** 16
- **Message:** `MSG_TYPE_OBSTACLE` (0x07)

**Obstacle protocol frame:**
```
Header (16 bytes):
  uint16  source_node_id
  uint8   num_detections
  uint8   sensor_id
  uint32  timestamp_ms
  uint32  scan_seq
  int16   range_max_mm
  int16   fov_deg

Per detection (8 bytes each):
  int16   range_mm
  int16   bearing_deg
  int16   velocity_mms
  uint8   confidence
  uint8   object_type
```

**Object types:**

| Value | Name | Description |
|-------|------|-------------|
| 0 | UNKNOWN | Unclassified detection |
| 1 | STATIC | Stationary object |
| 2 | MOVING | Moving object (has velocity) |
| 3 | BOAT | Another vessel |
| 4 | SHORE | Shoreline or land mass |
| 5 | DEBRIS | Floating debris |

---

### a02yyuw (`a02yyuw.h / .c`)

UART driver for the **DFRobot A02YYUW** waterproof ultrasonic distance sensor.

The A02YYUW measures range in one fixed direction using a simple 4-byte UART
protocol. Since the sensor has no angular output, each instance is configured
with a `mounting_angle_deg` that describes which direction it faces on the hull.
Up to four sensor instances can be active simultaneously at different mounting
angles to achieve multi-directional coverage.

The driver exposes `a02yyuw_scan_driver()` which is a drop-in `sensor_driver_fn`
for `obstacle_sense_register_driver()`.

**Sensor specifications:**

| Parameter | Value |
|-----------|-------|
| Interface | UART, 9600 baud |
| Supply voltage | 3.3 V or 5 V |
| Current draw | ~40 mA |
| Detection range | 30 mm – 4 500 mm |
| Update rate | ~10 Hz |
| Waterproof rating | IP67 |
| Angular output | None (point sensor) |

**Frame format (4 bytes):**
```
Byte 0: 0xFF        (header)
Byte 1: DATA_H      (distance high byte)
Byte 2: DATA_L      (distance low byte)
Byte 3: SUM         (checksum = (0xFF + DATA_H + DATA_L) & 0xFF)

Distance (mm) = DATA_H * 256 + DATA_L
Valid range: 30 to 4500 mm
```

**Wiring:**

| Wire colour | Connect to |
|-------------|-----------|
| Red | 3.3 V or 5 V |
| Black | GND |
| Blue | ESP32 UART RX pin (sensor TX) |
| Green | ESP32 UART TX pin (sensor RX) — usually unused |

**Multi-sensor layout for 360° coverage:**
```
            BOW (0°)
              [S0]
               ↑
               │
  [S3] (-90°)──┼──(+90°) [S1]
               │
               ↓
              [S2]
           STERN (180°)

S0: UART_NUM_1, RX=GPIO17, angle=   0° (bow)
S1: UART_NUM_2, RX=GPIO18, angle= +90° (starboard)
S2: requires additional UART,  angle= 180° (stern)
S3: requires additional UART,  angle= -90° (port)
```

**Activating the hardware driver:**

In `main.c`, uncomment the sensor block before `obstacle_sense_start()`:

```c
#include "a02yyuw.h"

// Initialize sensors (call before obstacle_sense_start)
a02yyuw_init(0, UART_NUM_1, GPIO_NUM_17, -1, 0);   // Bow sensor
a02yyuw_init(1, UART_NUM_2, GPIO_NUM_18, -1, 90);  // Starboard sensor

// Register with obstacle_sense (replaces mock driver)
obstacle_sense_register_driver(a02yyuw_scan_driver);
obstacle_sense_configure(4500, 120);  // 4.5 m range, 120° FOV per sensor
```

---

### avoidance (`avoidance.h / .c`)

Collision avoidance policy engine. Runs a 250 ms check loop that queries the
fused obstacle picture from `obstacle_sense` and transitions through a three-level
state machine:

```
DISABLED → CLEAR → WARNING → DANGER
               ↑_______________|
```

| State | Condition | Action |
|-------|-----------|--------|
| CLEAR | No obstacle within `warning_mm` | None |
| WARNING | Obstacle < `warning_mm` | Fire state callback; log alert |
| DANGER | Obstacle < `danger_mm` | Fire callback; auto-stop (leader); suggest alternate bearing |

When entering DANGER, the engine searches 8 candidate bearings
(0°, ±45°, ±90°, ±135°, 180°) for the first one that is clear within
`warning_mm`. The result is reported via the `avoidance_suggest_cb_t` callback.

Auto-stop only executes when `auto_stop = true` **and** this node is the swarm
leader, keeping the entire swarm synchronized rather than having individual nodes
act independently.

**Default thresholds:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `danger_mm` | 500 mm | Obstacle closer than this triggers DANGER |
| `warning_mm` | 1 500 mm | Obstacle closer than this triggers WARNING |
| `corridor_deg` | 40° | Path-check corridor total width (±20°) |
| `auto_stop` | true | Issue CMD_STOP when entering DANGER |

---

## Hardware Setup

### Minimum (mock sensor mode)

- 2× ESP32-S3 development boards
- USB cables for serial console and flashing

No physical distance sensors are required. The mock driver in `obstacle_sense`
lets you simulate obstacles via console commands for full swarm testing.

### With A02YYUW Sensors

**Single forward-facing sensor:**

```
ESP32-S3                  A02YYUW
──────────                ───────
3.3V or 5V ──────────────  Red
GND        ──────────────  Black
GPIO17 (RX)──────────────  Blue (sensor TX)
(TX unused)
```

**Dual sensor (bow + starboard):**

```
ESP32-S3     Sensor 0 (bow)   Sensor 1 (starboard)
3.3V     ──► Red              Red
GND      ──► Black            Black
GPIO17   ──► Blue             —
GPIO18   ──────────────────►  Blue
```

> **Note:** The ESP32-S3 has three hardware UART peripherals (UART0, UART1, UART2).
> UART0 is normally used by the serial console, leaving UART1 and UART2 for sensors.
> For more than two sensors, consider an external UART multiplexer.

---

## Building and Flashing

### Prerequisites

- ESP-IDF v5.5.x ([installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/))
- Two or more ESP32-S3 development boards
- USB cables

### Build

```bash
cd swarmLogic

# Build firmware (single binary for all nodes)
idf.py build
```

### Flash

```bash
# Flash Node 1 (adjust COM port as needed)
idf.py -p COM9 flash monitor

# In the serial console:
swarm> set_node_id 1
# Press Ctrl+T, Ctrl+R to restart

# Flash Node 2
idf.py -p COM7 flash monitor

swarm> set_node_id 2
# Press Ctrl+T, Ctrl+R to restart
```

### Expected Boot Output

```
I (442) MAIN: ================================================
I (442) MAIN:  AQUABOTS — ESP32-S3 Swarm Framework
I (442) MAIN: ================================================
I (442) MAIN: Node ID : 2
I (442) MAIN: MAC     : 10:b4:1d:d0:a5:f4
I (452) MEMBERSHIP: HELLO broadcast task started
I (452) LEADER_ELECTION: Leader election task started
I (452) COMMAND_ENGINE: Command engine started
I (452) FORMATION: Formation control started
I (452) OBSTACLE_SENSE: Scan task started (interval=100 ms)
I (452) OBSTACLE_SENSE: Broadcast task started (interval=500 ms)
I (452) AVOIDANCE: Avoidance started
I (1052) MAIN: *** NEW MEMBER DISCOVERED ***
I (1052) MAIN:     Node ID: 1
I (1072) MAIN: *** NEW LEADER ELECTED ***
I (1072) MAIN:     Leader Node ID: 2
```

---

## Node Provisioning

Each board must be assigned a unique node_id before it can participate in the
swarm. Node IDs persist through firmware updates and power cycles.

```bash
# Check current node_id
swarm> get_node_id

# Assign a node_id (1–254)
swarm> set_node_id 3

# Restart to activate
[Ctrl+T, Ctrl+R]

# Verify
swarm> info
```

**Rules:**
- Valid range: 1–254
- Higher node_id = leader (deterministic, no election messages needed)
- Each node in the swarm must have a unique ID
- ID 0 and 255 are reserved

---

## Console Command Reference

### Node and Network Status

| Command | Description |
|---------|-------------|
| `help` | List all available commands |
| `info` | Show node_id, MAC, and provisioning status |
| `get_node_id` | Display current node_id |
| `set_node_id <id>` | Assign node_id 1–254 (requires restart) |
| `members` | Show discovered swarm members with RSSI |
| `leader` | Show current leader and election state |
| `schedule` | Show TDMA slot schedule |
| `status` | Show current command execution state |

### Swarm Commands *(leader only)*

| Command | Description | Example |
|---------|-------------|---------|
| `goto <x> <y> [heading]` | Navigate swarm to waypoint (metres) | `goto 10 20 90` |
| `hold` | Hold current position | `hold` |
| `stop` | Emergency stop (motors off) | `stop` |
| `resume` | Resume last command | `resume` |

### Formation Control *(leader only)*

| Command | Description | Example |
|---------|-------------|---------|
| `formation <type>` | Set formation pattern | `formation wedge` |
| `spacing <mm>` | Set inter-node spacing in mm | `spacing 3000` |
| `fmove <dx> <dy>` | Translate entire formation (mm) | `fmove 1000 500` |
| `fstatus` | Show formation type and node assignments | `fstatus` |

Formation types: `line` `wedge` `circle` `column`

### Obstacle Sensing

| Command | Description | Example |
|---------|-------------|---------|
| `sensors` | Show sensor driver status and scan counters | `sensors` |
| `detections` | Show all local and remote obstacle detections | `detections` |
| `mock_add <range_mm> <bearing_deg>` | Inject a mock obstacle | `mock_add 2000 0` |
| `mock_clear` | Remove all mock obstacles | `mock_clear` |
| `path_check <bearing> [corridor] [range]` | Check if a heading is clear | `path_check 0` |

`path_check` defaults: corridor = 30°, range = 3 000 mm

### Collision Avoidance

| Command | Description | Example |
|---------|-------------|---------|
| `avoid_status` | Show avoidance state and current thresholds | `avoid_status` |
| `avoid_enable` | Enable the avoidance engine | `avoid_enable` |
| `avoid_disable` | Disable the avoidance engine | `avoid_disable` |
| `avoid_config <param> <value>` | Tune a threshold at runtime | `avoid_config danger_mm 400` |

`avoid_config` parameters: `danger_mm` `warning_mm` `corridor_deg` `auto_stop`

---

## Integration Guide

### Registering a Real Sensor

To replace mock mode with the A02YYUW hardware driver, add the following block
to `main.c` **before** the call to `obstacle_sense_start()`:

```c
#include "a02yyuw.h"

// One sensor, facing forward (0°), on UART1, RX = GPIO17
a02yyuw_init(0, UART_NUM_1, GPIO_NUM_17, -1, 0);

// Replace mock driver with the hardware driver
obstacle_sense_register_driver(a02yyuw_scan_driver);

// Optional: match sensor spec (4.5 m range, 120° FOV)
obstacle_sense_configure(4500, 120);
```

### Implementing a Custom Sensor Driver

Any sensor that can produce range/bearing measurements can be used. Implement
the `sensor_driver_fn` signature and register it:

```c
bool my_sensor_driver(obstacle_scan_t *scan)
{
    // Read from hardware
    int16_t range_mm = my_sensor_read();
    if (range_mm <= 0) {
        scan->num_detections = 0;
        return true;  // Empty scan is valid
    }

    scan->num_detections = 1;
    scan->detections[0].range_mm     = range_mm;
    scan->detections[0].bearing_deg  = 0;      // Sensor faces forward
    scan->detections[0].velocity_mms = 0;
    scan->detections[0].confidence   = 85;
    scan->detections[0].object_type  = OBSTACLE_TYPE_UNKNOWN;
    return true;
}

// Register before obstacle_sense_start()
obstacle_sense_register_driver(my_sensor_driver);
```

### Adding a Formation Pattern

```c
// 1. Add enum value in formation.h
typedef enum {
    FORMATION_NONE = 0,
    FORMATION_LINE,
    FORMATION_WEDGE,
    FORMATION_CIRCLE,
    FORMATION_COLUMN,
    FORMATION_DIAMOND,   // ← new pattern
    FORMATION_COUNT
} formation_type_t;

// 2. Implement calculator function
bool diamond_calc(int node_index, int total_nodes,
                  int32_t spacing_mm, formation_position_t *pos)
{
    // Assign positions for diamond arrangement
    // ...
    return true;
}

// 3. Register in formation_init() inside formation.c
formation_register_pattern(FORMATION_DIAMOND, "diamond", diamond_calc);
```

### Avoidance Callbacks

```c
void my_state_cb(avoidance_state_t state, const obstacle_detection_t *closest)
{
    if (state == AVOIDANCE_STATE_DANGER && closest) {
        // Log to mission computer, light warning LED, etc.
    }
}

void my_suggest_cb(int16_t bearing_deg, int16_t range_clear_mm)
{
    // Could feed this bearing into a higher-level autopilot
    printf("Suggested heading: %d°\n", bearing_deg);
}

// Register before avoidance_start()
avoidance_register_state_cb(my_state_cb);
avoidance_register_suggest_cb(my_suggest_cb);
```

---

## Testing and Verification

### Basic Connectivity

```bash
# On both nodes
swarm> info
# Should show provisioned node_id and MAC

swarm> members
# Should show 1 peer after ~2 seconds
```

### Leader Election

```bash
# On both nodes
swarm> leader
# Node with higher node_id should show "I am leader: YES"
# Lower node_id shows "I am leader: NO"
```

### Command Distribution

```bash
# On the LEADER node only
swarm> goto 100 200 90
# Both nodes should log "*** COMMAND RECEIVED ***"

swarm> status
# Both nodes should show "Command: GOTO, Status: EXECUTING"
```

### Formation Control

```bash
# On the LEADER node
swarm> formation line
# Both nodes should log "*** FORMATION CHANGED ***" and "*** POSITION ASSIGNED ***"

swarm> fstatus
# Shows current formation type and each node's assigned offset
```

### Obstacle Sensing

```bash
# On any node
swarm> sensors
# Should show: Running: YES, Mock mode: ENABLED

# Add mock obstacles
swarm> mock_add 2000 0      # 2 m straight ahead
swarm> mock_add 1500 45     # 1.5 m at 45°
swarm> mock_add 3000 -30    # 3 m at -30°

# Verify local detections
swarm> detections
# Shows 3 obstacles with range/bearing/type

# On the OTHER node — verify remote reception
swarm> detections
# Shows obstacles from the first node as "(remote)"

# Test path clearance
swarm> path_check 0
# Expected: "BLOCKED" (obstacle at 2000 mm @ 0°)

swarm> path_check 90
# Expected: "CLEAR" (no obstacles near 90°)

# Clean up
swarm> mock_clear
swarm> detections
# Expected: no detections
```

### Collision Avoidance

```bash
swarm> avoid_status
# Shows: State: CLEAR, thresholds, auto-stop setting

# Trigger a danger condition with a mock obstacle
swarm> mock_add 400 0       # 40 cm directly ahead (< 500 mm danger zone)

# Watch logs for:
# W AVOIDANCE: DANGER: 400 mm @ 0° (node X)
# W AVOIDANCE: Auto-stop issued to swarm
# I AVOIDANCE: Suggested alternate bearing: 90°

# Adjust thresholds at runtime
swarm> avoid_config danger_mm 300
swarm> avoid_config auto_stop 0     # Disable auto-stop

# Clean up
swarm> mock_clear
```

---

## Troubleshooting

### Nodes Don't Discover Each Other

1. Verify both nodes are provisioned: `get_node_id` (must return 1–254)
2. Both must be restarted after provisioning
3. Check "HELLO broadcast task started" appears in boot log
4. Verify distance < 30 m (indoor range is typically 10–50 m)
5. Check RSSI in `members` output — values worse than −80 dBm indicate weak signal

### No Leader Elected

1. Wait 5 seconds — leader election runs on a fixed interval
2. Verify both nodes appear in each other's `members` table
3. Confirm both nodes have different node_ids

### Command "Failed: ESP_ERR_INVALID_STATE"

Only the leader can originate commands. Run `leader` to find which node is
the leader, and issue commands from that node.

### Scan Count Stays at 0 in `sensors`

`obstacle_sense_start()` requires a valid node_id. If the node was not
provisioned at boot time, restart after provisioning.

### Remote Obstacles Not Appearing

1. Confirm both nodes are discovered (`members`)
2. Check that the remote node's scan count is incrementing (`sensors`)
3. Look for `*** REMOTE SCAN from node X ***` in the log
4. Remote scans expire after 2 seconds if broadcasts stop

### `path_check` Always Returns CLEAR

Add mock obstacles first (`mock_add 1000 0`). Confirm `detections` shows
them before running `path_check`. The default check range is 3 000 mm —
obstacles beyond that range will not block the path.

### A02YYUW Sensor Not Responding

1. Verify wiring: blue (sensor TX) → ESP32 UART RX pin
2. Confirm correct UART port and GPIO pin numbers in `a02yyuw_init()`
3. Check that no other peripheral shares the same UART port
4. Run `sensors` to confirm the driver is registered and not in mock mode
5. A stale reading appears in `a02yyuw_print_status()` if no valid frame
   has been received within 500 ms — verify sensor power (needs 3.3V or 5V)

---

## Performance Reference

### Network Timings

| Metric | Value |
|--------|-------|
| Ping round-trip time | 4–5 ms |
| RSSI at 1 m | −15 to −25 dBm |
| Peer discovery time | 0–4 seconds |
| Leader election time | < 5 seconds |
| HELLO broadcast interval | 2 000 ms |
| Beacon interval | 1 000 ms |
| TDMA slot duration | 50 ms/node |
| Command latency | < 50 ms |
| Obstacle scan rate (local) | 100 ms (10 Hz) |
| Obstacle broadcast rate | 500 ms (2 Hz) |
| Obstacle scan expiry | 2 000 ms |
| Avoidance check interval | 250 ms |

### Resource Usage (Per Node)

| Resource | Usage |
|----------|-------|
| Flash (firmware) | ~870 KB |
| RAM (all modules) | ~16 KB |
| CPU | < 10% (all tasks combined) |
| Network traffic | ~150–200 bytes/sec |

### Scalability

| Swarm size | TDMA cycle | Network traffic |
|------------|------------|-----------------|
| 2 nodes | 100 ms | ~100 bytes/sec |
| 10 nodes | 500 ms | ~300 bytes/sec |
| 50 nodes | 2 500 ms | ~800 bytes/sec |

Maximum supported swarm size: **50 nodes** (set by `SWARM_MAX_NODES` in
`swarm_protocol.h` and `FORMATION_MAX_NODES` implied by formation arrays).

### RSSI Signal Quality Guide

```
−10 to −30 dBm  → Excellent  (< 1 metre)
−30 to −50 dBm  → Good       (1–10 metres)
−50 to −70 dBm  → Fair       (10–50 metres)
−70 to −90 dBm  → Weak       (50–100 metres, marginal)
```

---

## File Structure

```
main/
├── main.c                  Entry point; module init, callbacks, console setup
├── swarm_protocol.h        Packet format definitions and message type enum
├── swarm_transport.h / .c  ESP-NOW transport abstraction
├── node_config.h / .c      Persistent node identity (NVS) and serial console
├── membership.h / .c       Peer discovery via HELLO/BEACON broadcasts
├── leader_election.h / .c  Leader election and TDMA schedule distribution
├── scheduler.h / .c        TDMA slot scheduling utilities
├── command_engine.h / .c   Swarm command distribution and execution
├── formation.h / .c        Formation control with pluggable pattern system
├── obstacle_sense.h / .c   Sensor abstraction and swarm-wide scan fusion
├── a02yyuw.h / .c          DFRobot A02YYUW ultrasonic sensor UART driver
├── avoidance.h / .c        Collision avoidance policy engine
└── CMakeLists.txt          ESP-IDF component build configuration
```

---

## Hardware Compatibility

| Board | Status |
|-------|--------|
| ESP32-S3 | ✅ Primary target, fully tested |
| ESP32 (original) | ✅ Should work (untested) |
| ESP32-C3 | ✅ Should work (untested) |
| ESP32-S2 | ✅ Should work (untested) |
| ESP32-C6 | ⚠️ Requires WiFi driver changes |
| ESP32-H2 | ❌ No WiFi radio |

---

## License

- **ESP-IDF:** Apache 2.0 (Espressif Systems)
- **ESP-NOW:** Proprietary (Espressif Systems)
- **FreeRTOS:** MIT

---

*AQUABOTS Swarm Framework — ESP32-S3 Edition*
*Platform: ESP32-S3 with ESP-IDF v5.5.x*
*Last updated: February 2026*
