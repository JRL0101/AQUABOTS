/**
 * formation.h
 * 
 * Formation control with extensible pattern system.
 * Coordinated spatial formations: line, wedge, circle, column.
 *
 * EXTENSIBILITY:
 * To add a new formation pattern:
 * 1. Add enum value to formation_type_t
 * 2. Create calculator function: bool my_pattern_calc(int node_index, int total_nodes, 
 *                                                     int32_t spacing, formation_position_t *pos)
 * 3. Register in formation_init(): formation_register_pattern(FORMATION_MY_PATTERN, 
 *                                                             "my_pattern", my_pattern_calc)
 */

#ifndef FORMATION_H
#define FORMATION_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define FORMATION_DEFAULT_SPACING_MM    2000    // 2 meters between nodes
#define FORMATION_MIN_SPACING_MM        500     // 0.5 meter minimum
#define FORMATION_MAX_SPACING_MM        10000   // 10 meters maximum
#define FORMATION_MAX_PATTERNS          16      // Maximum registered patterns

// ============================================================================
// TYPES
// ============================================================================

/**
 * Formation types - add new types here
 * IMPORTANT: Keep FORMATION_NONE as 0 and FORMATION_COUNT at end
 */
typedef enum {
    FORMATION_NONE = 0,     // No formation (free movement)
    FORMATION_LINE,         // Single file, leader at front
    FORMATION_WEDGE,        // V-shape, leader at point
    FORMATION_CIRCLE,       // Ring around center point
    FORMATION_COLUMN,       // Side-by-side parallel columns
    // Add new formation types above this line
    FORMATION_COUNT         // Must be last - used for validation
} formation_type_t;

/**
 * Position offset for a node within formation
 * All values relative to formation center
 */
typedef struct {
    int32_t x_mm;           // X offset from formation center (mm)
    int32_t y_mm;           // Y offset from formation center (mm)
    int16_t heading_deg;    // Heading offset from formation heading (degrees)
} formation_position_t;

/**
 * Formation configuration (shared across swarm)
 */
typedef struct {
    formation_type_t type;          // Current formation type
    uint8_t node_count;             // Number of nodes in formation
    int32_t spacing_mm;             // Distance between nodes
    int32_t center_x_mm;            // Formation center X position
    int32_t center_y_mm;            // Formation center Y position
    int16_t center_heading_deg;     // Formation heading
    uint32_t config_seq;            // Sequence number for change detection
} formation_config_t;

/**
 * Pattern calculator function pointer type
 * 
 * @param node_index    This node's index in the formation (0 = leader, 1..n = followers)
 * @param total_nodes   Total number of nodes in formation
 * @param spacing_mm    Spacing between nodes in mm
 * @param pos           Output: calculated position offset
 * @return              true if position calculated successfully
 * 
 * To add a new pattern, implement this function signature.
 */
typedef bool (*pattern_calculator_fn)(int node_index, int total_nodes, 
                                      int32_t spacing_mm, formation_position_t *pos);

/**
 * Pattern registration entry
 */
typedef struct {
    formation_type_t type;
    const char *name;
    pattern_calculator_fn calculator;
    bool registered;
} pattern_entry_t;

// ============================================================================
// CALLBACK TYPES
// ============================================================================

/**
 * Called when formation configuration changes
 */
typedef void (*formation_changed_cb_t)(const formation_config_t *config);

/**
 * Called when this node's position assignment changes
 */
typedef void (*position_assigned_cb_t)(const formation_position_t *position, int node_index);

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * Initialize formation module
 * Registers built-in patterns and creates mutex
 */
esp_err_t formation_init(void);

/**
 * Start formation protocol (register transport handler)
 */
esp_err_t formation_start(void);

/**
 * Stop formation protocol
 */
void formation_stop(void);

/**
 * Set formation type (leader only, broadcasts to swarm)
 * 
 * @param type  Formation type to set
 * @return      ESP_OK on success, ESP_ERR_INVALID_STATE if not leader
 */
esp_err_t formation_set_type(formation_type_t type);

/**
 * Set spacing between nodes (leader only, broadcasts to swarm)
 * 
 * @param spacing_mm    Spacing in millimeters (500-10000)
 * @return              ESP_OK on success
 */
esp_err_t formation_set_spacing(int32_t spacing_mm);

/**
 * Move entire formation to new position (leader only)
 * 
 * @param x_mm          Target X position in mm
 * @param y_mm          Target Y position in mm
 * @param heading_deg   Target heading in degrees
 * @return              ESP_OK on success
 */
esp_err_t formation_move(int32_t x_mm, int32_t y_mm, int16_t heading_deg);

/**
 * Get this node's assigned position in current formation
 * 
 * @param pos   Output: position offset from formation center
 * @return      ESP_OK if position assigned, ESP_ERR_NOT_FOUND if not in formation
 */
esp_err_t formation_get_my_position(formation_position_t *pos);

/**
 * Get this node's index in the formation (0 = leader position)
 * 
 * @return  Node index (0..n-1), or -1 if not assigned
 */
int formation_get_my_index(void);

/**
 * Get current formation configuration
 * 
 * @param config    Output: current configuration
 * @return          ESP_OK on success
 */
esp_err_t formation_get_config(formation_config_t *config);

/**
 * Check if formation is active
 * 
 * @return  true if a formation is set (type != FORMATION_NONE)
 */
bool formation_is_active(void);

// ============================================================================
// PATTERN REGISTRATION API (for extensibility)
// ============================================================================

/**
 * Register a custom formation pattern
 * Call during initialization to add new patterns
 * 
 * @param type          Formation type enum value
 * @param name          Human-readable name (for console/logging)
 * @param calculator    Position calculator function
 * @return              ESP_OK on success, ESP_ERR_NO_MEM if registry full
 */
esp_err_t formation_register_pattern(formation_type_t type, const char *name,
                                     pattern_calculator_fn calculator);

/**
 * Get pattern name string
 * 
 * @param type  Formation type
 * @return      Name string, or "UNKNOWN" if not registered
 */
const char* formation_type_to_string(formation_type_t type);

/**
 * Parse formation type from string
 * 
 * @param name  Formation name string (case-insensitive)
 * @return      Formation type, or FORMATION_NONE if not found
 */
formation_type_t formation_type_from_string(const char *name);

// ============================================================================
// CALLBACK REGISTRATION
// ============================================================================

/**
 * Register callback for formation changes
 */
void formation_register_changed_cb(formation_changed_cb_t cb);

/**
 * Register callback for position assignment changes
 */
void formation_register_position_cb(position_assigned_cb_t cb);

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

/**
 * Register formation console commands
 * Called from main.c after console initialization
 */
void formation_register_commands(void);

/**
 * Print formation status to console
 */
void formation_print_status(void);

#endif // FORMATION_H
