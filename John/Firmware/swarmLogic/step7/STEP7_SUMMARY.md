# Step 7: Obstacle Sensor Scaffolding - Summary

**Status:** Implementation Complete  
**Estimated Lines:** ~750  
**New Files:** obstacle_sense.h, obstacle_sense.c (updated main.c)

---

## What Was Built

### Driver Abstraction System
- **Pluggable sensor drivers** via function pointers
- **Mock driver** for testing without hardware
- **Easy hardware integration** - just implement one function

### Key Features
- Local obstacle scanning at 10 Hz
- Swarm-wide obstacle sharing at 2 Hz
- Remote scan storage from all nodes
- Path clearance checking
- Closest obstacle detection across swarm

### Protocol
- **MSG_TYPE_OBSTACLE** broadcasts
- Compact packed format (8 bytes per detection)
- Sequence numbers for change detection
- 2-second scan timeout for stale data

---

## Console Commands (5 New)

| Command | Description |
|---------|-------------|
| `sensors` | Show sensor status |
| `detections` | Show all detected obstacles |
| `mock_add <range> <bearing>` | Add mock obstacle for testing |
| `mock_clear` | Clear all mock obstacles |
| `path_check <bearing>` | Check if path is clear |

---

## Key APIs

```c
// Register custom sensor driver (for Step 8)
esp_err_t obstacle_sense_register_driver(sensor_driver_fn driver);

// Get closest obstacle across entire swarm
esp_err_t obstacle_sense_get_closest(obstacle_detection_t *detection, uint16_t *node_id);

// Check if path is clear
bool obstacle_sense_path_clear(int16_t bearing_deg, int16_t corridor_deg, int16_t range_mm);

// Mock mode for testing
esp_err_t obstacle_sense_mock_add_simple(int16_t range_mm, int16_t bearing_deg);
```

---

## Integrating Real Hardware (Step 8 Preview)

```c
// Your mmWave driver function
bool my_radar_driver(obstacle_scan_t *scan) {
    // Read from hardware
    // Fill scan->detections[]
    // Set scan->num_detections
    return true;
}

// Register during init
obstacle_sense_register_driver(my_radar_driver);
```

---

## Recommended Sensors for Through-Hull Mounting

### 🏆 BEST CHOICE: Hi-Link LD2450

| Spec | Value |
|------|-------|
| **Price** | ~$15-20 |
| **Frequency** | 24 GHz FMCW |
| **Range** | 0.2 - 6 meters |
| **Field of View** | 120° horizontal |
| **Interface** | UART (115200 baud) |
| **Power** | 5V, ~80mA |
| **Output** | Up to 3 targets with X,Y coordinates |
| **Hull Penetration** | ✅ Excellent through fiberglass |

**Why it's perfect:**
- 24 GHz penetrates fiberglass easily
- Provides range AND bearing (X,Y coordinates)
- Multiple target tracking (up to 3)
- Very affordable
- Simple UART protocol
- Low power consumption

**Where to buy:**
- AliExpress: Search "LD2450 radar"
- Amazon: "HLK-LD2450"

---

### Alternative Options

#### Acconeer XM125 (~$50-60)

| Spec | Value |
|------|-------|
| **Frequency** | 60 GHz pulsed coherent |
| **Range** | 0.02 - 20 meters |
| **Accuracy** | ±1mm |
| **Interface** | SPI/I2C |
| **Hull Penetration** | ✅ Good through fiberglass |

**Pros:** Extremely precise ranging, long range  
**Cons:** Higher cost, more complex integration, single point (no angle)

#### TI IWR1443BOOST (~$300)

| Spec | Value |
|------|-------|
| **Frequency** | 76-81 GHz |
| **Range** | Up to 80 meters |
| **Resolution** | ~4cm range, ~15° angle |
| **Interface** | UART/SPI |
| **Hull Penetration** | ⚠️ May need testing |

**Pros:** Professional grade, excellent performance  
**Cons:** Expensive, complex, may not penetrate thick fiberglass

#### Ultrasonic (JSN-SR04T waterproof) (~$5)

| Spec | Value |
|------|-------|
| **Range** | 0.2 - 4.5 meters |
| **Frequency** | 40 kHz |
| **Interface** | GPIO trigger/echo |
| **Hull Penetration** | ❌ Poor through fiberglass |

**Pros:** Very cheap, waterproof version available  
**Cons:** Won't work through hull, single point, affected by waves

---

### Sensor Comparison for ASV Use

| Sensor | Price | Hull Mount | Range | Multi-Target | Recommendation |
|--------|-------|------------|-------|--------------|----------------|
| **LD2450** | $15 | ✅ Yes | 6m | ✅ 3 targets | ⭐ BEST VALUE |
| **LD2410** | $5 | ✅ Yes | 6m | ❌ Single | Good for proximity |
| XM125 | $50 | ✅ Yes | 20m | ❌ Single | Precise ranging |
| IWR1443 | $300 | ⚠️ Test | 80m | ✅ Many | Professional |
| Ultrasonic | $5 | ❌ No | 4m | ❌ Single | External mount only |

---

### LD2450 Mounting Recommendations

```
┌─────────────────────────────────────┐
│         FIBERGLASS HULL             │
│         (3-6mm thick)               │
├─────────────────────────────────────┤
│                                     │
│    ┌─────────┐                      │
│    │ LD2450  │  ← Mount flat against│
│    │  RADAR  │    inside of hull    │
│    └─────────┘                      │
│         ↓                           │
│    Detection cone: 120° × 120°      │
│                                     │
└─────────────────────────────────────┘
```

**Mounting tips:**
1. Mount sensor flat against hull interior
2. Ensure no metal between sensor and hull
3. Clean hull surface for best transmission
4. Test with actual hull material before permanent install
5. Consider 2-4 sensors for 360° coverage

---

### Multi-Sensor Layout for 360° Coverage

```
         FRONT (Bow)
            [S1]
             ↑
             │
    [S4] ←───┼───→ [S2]
             │
             ↓
            [S3]
         REAR (Stern)

S1: Forward detection (primary)
S2: Starboard detection
S3: Rear detection
S4: Port detection
```

**Configuration for 4 sensors:**
- Each sensor: LD2450 ($15 each)
- Total cost: ~$60-80
- Full 360° coverage
- Each connected to separate UART

---

## Protocol Details

### Message Format

```c
// Header (16 bytes)
typedef struct {
    uint16_t source_node_id;
    uint8_t num_detections;
    uint8_t sensor_id;
    uint32_t timestamp_ms;
    uint32_t scan_seq;
    int16_t range_max_mm;
    int16_t fov_deg;
} obstacle_payload_t;

// Per-detection (8 bytes each)
typedef struct {
    int16_t range_mm;
    int16_t bearing_deg;
    int16_t velocity_mms;
    uint8_t confidence;
    uint8_t object_type;
} obstacle_detection_packed_t;
```

---

## Testing Workflow

```bash
# 1. Check sensor status
swarm> sensors

# 2. Add mock obstacles
swarm> mock_add 2000 0      # 2m ahead
swarm> mock_add 1500 45     # 1.5m at 45°
swarm> mock_add 3000 -30    # 3m at -30°

# 3. View detections
swarm> detections

# 4. Check path clearance
swarm> path_check 0         # Is straight ahead clear?
swarm> path_check 45        # Is 45° clear?

# 5. Clear mock data
swarm> mock_clear
```

---

## Files Changed

| File | Action | Description |
|------|--------|-------------|
| `obstacle_sense.h` | **Replace** | Full header with driver API |
| `obstacle_sense.c` | **Replace** | Full implementation (~650 lines) |
| `main.c` | **Update** | Add obstacle callbacks and start |
| `swarm_protocol.h` | **Add** | MSG_TYPE_OBSTACLE = 0x07 |

---

## Next Step Preview

**Step 8: mmWave Radar Integration**
- LD2450 UART driver
- Real obstacle detection
- Sensor calibration
- Multi-sensor fusion
