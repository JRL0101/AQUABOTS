/**
 * a02yyuw.h
 *
 * Driver for the DFRobot A02YYUW waterproof ultrasonic distance sensor.
 *
 * The A02YYUW communicates over UART at 9600 baud using a simple 4-byte
 * frame format. It measures distance in one fixed direction (no angle
 * output), so each sensor instance is configured with a mounting angle
 * that describes which direction it faces relative to the vehicle's bow.
 *
 * Multiple sensor instances can be initialized at different mounting angles
 * to achieve wider coverage (e.g., four sensors at 0°, 90°, 180°, 270° for
 * full 360° coverage). All initialized sensors are fused into a single
 * obstacle scan when a02yyuw_scan_driver() is called, making the driver
 * a drop-in replacement for the mock driver in the obstacle_sense module.
 *
 * Frame format (4 bytes):
 *   [0] 0xFF        Header byte
 *   [1] DATA_H      High byte of distance
 *   [2] DATA_L      Low byte of distance
 *   [3] SUM         Checksum = (0xFF + DATA_H + DATA_L) & 0xFF
 *
 * Distance (mm) = DATA_H * 256 + DATA_L
 * Valid range: 30 mm to 4500 mm (3 cm to 4.5 m)
 *
 * Hardware wiring:
 *   - Red   → 3.3V or 5V
 *   - Black → GND
 *   - Blue  → UART RX pin on ESP32 (sensor TX)
 *   - Green → UART TX pin on ESP32 (sensor RX) — usually unused
 */

#ifndef A02YYUW_H
#define A02YYUW_H

#include "esp_err.h"
#include "driver/uart.h"
#include "obstacle_sense.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define A02YYUW_MAX_SENSORS         4       // Maximum simultaneous sensor instances
#define A02YYUW_BAUD_RATE           9600    // Fixed UART baud rate
#define A02YYUW_RANGE_MIN_MM        30      // Minimum valid range (3 cm)
#define A02YYUW_RANGE_MAX_MM        4500    // Maximum valid range (4.5 m)
#define A02YYUW_FRAME_LEN           4       // Bytes per data frame
#define A02YYUW_FRAME_HEADER        0xFF    // Start-of-frame marker
#define A02YYUW_STALE_MS            500     // Discard readings older than this
#define A02YYUW_UART_BUF_SIZE       256     // UART RX ring buffer size
#define A02YYUW_CONFIDENCE          90      // Detection confidence reported (0-100)

// ============================================================================
// SENSOR INSTANCE
// ============================================================================

/**
 * State for one A02YYUW sensor instance.
 * Populated by a02yyuw_init(); do not modify directly.
 */
typedef struct {
    uart_port_t uart_port;          // ESP32 UART peripheral (UART_NUM_1, etc.)
    int rx_pin;                     // GPIO number connected to sensor TX (blue wire)
    int tx_pin;                     // GPIO number connected to sensor RX (green wire), or -1
    int16_t mounting_angle_deg;     // Direction sensor faces: 0=bow, 90=starboard, 180=stern, -90=port
    int16_t last_range_mm;          // Most recent valid range reading
    uint32_t last_update_ms;        // esp_timer timestamp of last valid frame
    bool initialized;               // True after successful a02yyuw_init()
} a02yyuw_instance_t;

// ============================================================================
// API
// ============================================================================

/**
 * Initialize one sensor instance.
 *
 * Configures and installs the UART driver for the specified port and GPIO
 * pins. The mounting_angle_deg is stored and used as the bearing in obstacle
 * scan reports — it should match the physical direction the sensor faces on
 * the vehicle hull (0° = bow, 90° = starboard, 180°/−180° = stern, −90° = port).
 *
 * Call once per sensor during application startup before calling
 * obstacle_sense_register_driver(a02yyuw_scan_driver).
 *
 * @param instance_index    Sensor slot (0 to A02YYUW_MAX_SENSORS-1)
 * @param uart_port         UART peripheral (e.g., UART_NUM_1)
 * @param rx_pin            GPIO for UART RX (connect to sensor blue wire)
 * @param tx_pin            GPIO for UART TX, or -1 if not needed
 * @param mounting_angle_deg Direction sensor faces relative to bow (-180 to +180)
 * @return ESP_OK on success
 */
esp_err_t a02yyuw_init(int instance_index, uart_port_t uart_port,
                        int rx_pin, int tx_pin, int16_t mounting_angle_deg);

/**
 * Read the most recent valid range from one sensor instance.
 *
 * Returns 0 if the sensor has not yet produced a valid reading or if the
 * last reading is older than A02YYUW_STALE_MS.
 *
 * @param instance_index    Sensor slot (0 to A02YYUW_MAX_SENSORS-1)
 * @return Range in mm, or 0 if unavailable
 */
int16_t a02yyuw_read_range(int instance_index);

/**
 * Drain UART buffers for all initialized sensors and update stored ranges.
 *
 * Processes all bytes currently waiting in each sensor's UART RX buffer,
 * searches for valid 4-byte frames, and updates last_range_mm / last_update_ms
 * on successful parse. Called automatically by a02yyuw_scan_driver().
 */
void a02yyuw_update_all(void);

/**
 * Sensor driver function compatible with obstacle_sense_register_driver().
 *
 * Calls a02yyuw_update_all(), then builds an obstacle_scan_t where each
 * initialized sensor contributes one detection entry at its mounting angle.
 * Sensors with stale or out-of-range readings are omitted.
 *
 * Register with:
 *   obstacle_sense_register_driver(a02yyuw_scan_driver);
 *
 * @param scan  Output scan struct to fill
 * @return      true always (empty scan is valid)
 */
bool a02yyuw_scan_driver(obstacle_scan_t *scan);

/**
 * Return the number of successfully initialized sensor instances.
 */
int a02yyuw_get_count(void);

/**
 * Get a read-only pointer to a specific instance struct (for diagnostics).
 *
 * @param instance_index  Sensor slot (0 to A02YYUW_MAX_SENSORS-1)
 * @return Pointer to instance, or NULL if index out of range
 */
const a02yyuw_instance_t *a02yyuw_get_instance(int instance_index);

/**
 * Print a formatted status table for all initialized sensors to stdout.
 */
void a02yyuw_print_status(void);

#endif // A02YYUW_H
