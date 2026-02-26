/**
 * obstacle_sense.h
 * 
 * Obstacle sensor interface with driver abstraction.
 * Step 7: Scaffolding with mock data for testing.
 * Step 8: Will add real mmWave radar driver.
 * 
 * EXTENSIBILITY:
 * To add a new sensor:
 * 1. Create driver function matching sensor_driver_fn signature
 * 2. Call obstacle_sense_register_driver() during init
 * 3. Driver will be called at OBSTACLE_SCAN_INTERVAL_MS rate
 */

#ifndef OBSTACLE_SENSE_H
#define OBSTACLE_SENSE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define OBSTACLE_MAX_DETECTIONS         16      // Max obstacles per scan
#define OBSTACLE_MAX_NODES              16      // Max nodes to track scans from
#define OBSTACLE_SCAN_INTERVAL_MS       100     // Local scan rate (10 Hz)
#define OBSTACLE_BROADCAST_INTERVAL_MS  500     // Share with swarm (2 Hz)
#define OBSTACLE_SCAN_TIMEOUT_MS        2000    // Consider scan stale after this
#define OBSTACLE_DEFAULT_RANGE_MAX_MM   5000    // 5 meter default max range

// ============================================================================
// TYPES
// ============================================================================

/**
 * Object classification types
 */
typedef enum {
    OBSTACLE_TYPE_UNKNOWN = 0,  // Unclassified
    OBSTACLE_TYPE_STATIC,       // Stationary object
    OBSTACLE_TYPE_MOVING,       // Moving object (has velocity)
    OBSTACLE_TYPE_BOAT,         // Another vessel (if identifiable)
    OBSTACLE_TYPE_SHORE,        // Shoreline/land mass
    OBSTACLE_TYPE_DEBRIS        // Floating debris
} obstacle_type_t;

/**
 * Single obstacle detection
 */
typedef struct {
    int16_t range_mm;           // Distance to obstacle (mm), 0 = invalid
    int16_t bearing_deg;        // Angle relative to heading (-180 to +180)
    int16_t velocity_mms;       // Radial velocity (mm/s), positive = approaching
    uint8_t confidence;         // Detection confidence (0-100%)
    uint8_t object_type;        // obstacle_type_t
} obstacle_detection_t;

/**
 * Complete scan from one sensor/node
 */
typedef struct {
    uint16_t source_node_id;    // Node that detected these obstacles
    uint8_t num_detections;     // Number of valid detections (0 to OBSTACLE_MAX_DETECTIONS)
    uint8_t sensor_id;          // Sensor index if node has multiple sensors
    uint32_t timestamp_ms;      // Scan timestamp (esp_log_timestamp)
    uint32_t scan_seq;          // Sequence number for change detection
    int16_t sensor_range_max_mm;// Max range of this sensor
    int16_t sensor_fov_deg;     // Field of view in degrees
    obstacle_detection_t detections[OBSTACLE_MAX_DETECTIONS];
} obstacle_scan_t;

/**
 * Sensor status
 */
typedef struct {
    bool driver_registered;     // Is a driver registered?
    bool running;               // Is scanning active?
    bool mock_enabled;          // Is mock mode active?
    uint32_t scan_count;        // Total scans performed
    uint32_t broadcast_count;   // Total broadcasts sent
    uint32_t last_scan_ms;      // Timestamp of last scan
    int16_t range_max_mm;       // Configured max range
    int16_t fov_deg;            // Configured field of view
} obstacle_sense_status_t;

/**
 * Sensor driver function pointer
 * 
 * Implement this to add a new sensor type:
 * - Called at OBSTACLE_SCAN_INTERVAL_MS rate
 * - Fill scan->detections[] with detected obstacles
 * - Set scan->num_detections to number of valid entries
 * - Return true if scan successful, false on error
 * 
 * @param scan  Output: fill with detected obstacles
 * @return      true if scan successful
 */
typedef bool (*sensor_driver_fn)(obstacle_scan_t *scan);

// ============================================================================
// CALLBACKS
// ============================================================================

/**
 * Called when new local scan is available
 */
typedef void (*obstacle_scan_cb_t)(const obstacle_scan_t *scan);

/**
 * Called when obstacle scan received from another node
 */
typedef void (*obstacle_remote_cb_t)(uint16_t node_id, const obstacle_scan_t *scan);

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * Initialize obstacle sensing module
 * Registers mock driver by default
 */
esp_err_t obstacle_sense_init(void);

/**
 * Start obstacle scanning and broadcasting
 */
esp_err_t obstacle_sense_start(void);

/**
 * Stop obstacle scanning
 */
void obstacle_sense_stop(void);

/**
 * Register custom sensor driver
 * Call this to replace mock driver with real hardware
 * 
 * @param driver    Driver function to call for each scan
 * @return          ESP_OK on success
 */
esp_err_t obstacle_sense_register_driver(sensor_driver_fn driver);

/**
 * Configure sensor parameters
 * 
 * @param range_max_mm  Maximum detection range in mm
 * @param fov_deg       Field of view in degrees
 */
void obstacle_sense_configure(int16_t range_max_mm, int16_t fov_deg);

/**
 * Get latest local scan (from this node's sensor)
 * 
 * @param scan  Output: latest scan data
 * @return      ESP_OK if scan available, ESP_ERR_NOT_FOUND if no scan yet
 */
esp_err_t obstacle_sense_get_local_scan(obstacle_scan_t *scan);

/**
 * Get latest scan from a specific node
 * 
 * @param node_id   Node to get scan from
 * @param scan      Output: scan data from that node
 * @return          ESP_OK if found, ESP_ERR_NOT_FOUND if not available
 */
esp_err_t obstacle_sense_get_node_scan(uint16_t node_id, obstacle_scan_t *scan);

/**
 * Get all known scans (local + remote)
 * 
 * @param scans     Output array
 * @param max_scans Size of output array
 * @return          Number of scans copied
 */
int obstacle_sense_get_all_scans(obstacle_scan_t *scans, int max_scans);

/**
 * Get closest obstacle across all known scans
 * 
 * @param detection Output: closest obstacle
 * @param node_id   Output: which node detected it (can be NULL)
 * @return          ESP_OK if obstacle found, ESP_ERR_NOT_FOUND if clear
 */
esp_err_t obstacle_sense_get_closest(obstacle_detection_t *detection, uint16_t *node_id);

/**
 * Check if path is clear in a direction
 * 
 * @param bearing_deg   Direction to check (-180 to +180)
 * @param corridor_deg  Width of corridor to check (e.g., 30 = ±15°)
 * @param range_mm      How far to check
 * @return              true if clear, false if obstacle detected
 */
bool obstacle_sense_path_clear(int16_t bearing_deg, int16_t corridor_deg, int16_t range_mm);

/**
 * Get module status
 */
void obstacle_sense_get_status(obstacle_sense_status_t *status);

// ============================================================================
// CALLBACK REGISTRATION
// ============================================================================

/**
 * Register callback for local scans
 */
void obstacle_sense_register_scan_cb(obstacle_scan_cb_t cb);

/**
 * Register callback for remote scans (from other nodes)
 */
void obstacle_sense_register_remote_cb(obstacle_remote_cb_t cb);

// ============================================================================
// MOCK MODE (for testing without hardware)
// ============================================================================

/**
 * Enable/disable mock mode
 * When enabled, mock driver generates fake obstacle data
 */
void obstacle_sense_mock_enable(bool enable);

/**
 * Check if mock mode is active
 */
bool obstacle_sense_mock_is_enabled(void);

/**
 * Add a mock obstacle
 * 
 * @param range_mm      Distance in mm
 * @param bearing_deg   Angle in degrees (-180 to +180)
 * @param velocity_mms  Radial velocity (positive = approaching)
 * @param type          Obstacle type
 * @return              ESP_OK on success, ESP_ERR_NO_MEM if full
 */
esp_err_t obstacle_sense_mock_add(int16_t range_mm, int16_t bearing_deg,
                                   int16_t velocity_mms, obstacle_type_t type);

/**
 * Add simple mock obstacle (stationary, unknown type)
 */
esp_err_t obstacle_sense_mock_add_simple(int16_t range_mm, int16_t bearing_deg);

/**
 * Clear all mock obstacles
 */
void obstacle_sense_mock_clear(void);

/**
 * Get number of mock obstacles configured
 */
int obstacle_sense_mock_count(void);

// ============================================================================
// UTILITIES
// ============================================================================

/**
 * Get obstacle type as string
 */
const char* obstacle_type_to_string(obstacle_type_t type);

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

/**
 * Register console commands
 */
void obstacle_sense_register_commands(void);

/**
 * Print sensor status to console
 */
void obstacle_sense_print_status(void);

/**
 * Print all detected obstacles to console
 */
void obstacle_sense_print_detections(void);

#endif // OBSTACLE_SENSE_H
