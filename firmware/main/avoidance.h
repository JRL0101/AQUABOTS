/**
 * avoidance.h
 *
 * Collision avoidance policy engine for the AQUABOTS swarm framework.
 *
 * The avoidance module runs a lightweight periodic task that consumes
 * fused obstacle data from obstacle_sense (both local and swarm-wide) and
 * takes autonomous action when objects enter configured safety zones:
 *
 *   CLEAR   — No obstacle within the warning range. Normal operation.
 *   WARNING — Obstacle within warning_mm. Application is notified via callback.
 *             No automatic action is taken; the operator can respond manually.
 *   DANGER  — Obstacle within danger_mm. If auto_stop is enabled and this node
 *             is the swarm leader, a STOP command is issued to the entire swarm.
 *             The engine simultaneously searches for a clear alternate bearing
 *             and reports it via the suggest callback so navigation can resume.
 *
 * The module is formation-aware in the sense that it only issues commands when
 * this node is the leader, keeping the entire swarm synchronized. Follower
 * nodes transition through warning/danger states and fire callbacks but do not
 * send independent commands.
 *
 * All thresholds and behaviors are runtime-configurable via console commands or
 * the avoidance_set_config() API. The module can also be disabled entirely
 * (e.g., during dock operations) with avoidance_disable() or via the
 * `avoid_disable` console command.
 *
 * Console commands registered by avoidance_register_commands():
 *   avoid_status               Print current state and configuration
 *   avoid_enable               Enable the avoidance engine
 *   avoid_disable              Disable the avoidance engine
 *   avoid_config <param> <val> Tune parameters at runtime
 */

#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include "esp_err.h"
#include "obstacle_sense.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// CONFIGURATION DEFAULTS
// ============================================================================

/** How often the avoidance task runs its check loop (ms). */
#define AVOIDANCE_CHECK_INTERVAL_MS     250

/** Range below which DANGER state is entered (mm). Default 50 cm. */
#define AVOIDANCE_DANGER_MM_DEFAULT     500

/** Range below which WARNING state is entered (mm). Default 1.5 m. */
#define AVOIDANCE_WARNING_MM_DEFAULT    1500

/**
 * Width of the path-clearance corridor used both for forward-path checks
 * and when searching for an alternate safe bearing (degrees, total width).
 * A value of 40 means the corridor extends ±20° from the checked bearing.
 */
#define AVOIDANCE_CORRIDOR_DEG_DEFAULT  40

/** Number of candidate alternate bearings to evaluate when searching for a
 *  clear path. Candidates are evenly distributed around the compass. */
#define AVOIDANCE_ALT_BEARING_COUNT     8

// ============================================================================
// STATE
// ============================================================================

/**
 * Avoidance engine operational state.
 */
typedef enum {
    AVOIDANCE_STATE_DISABLED = 0,   /**< Engine is off; no checks or commands. */
    AVOIDANCE_STATE_CLEAR,          /**< No obstacles within warning range. */
    AVOIDANCE_STATE_WARNING,        /**< Obstacle detected within warning range. */
    AVOIDANCE_STATE_DANGER,         /**< Obstacle within danger range; action taken. */
} avoidance_state_t;

// ============================================================================
// RUNTIME CONFIGURATION
// ============================================================================

/**
 * Runtime-tunable avoidance parameters.
 * Obtain current values with avoidance_get_config() and apply changes
 * with avoidance_set_config() or via the `avoid_config` console command.
 */
typedef struct {
    bool    enabled;            /**< Master enable; false freezes all state transitions. */
    int16_t danger_mm;          /**< Obstacle range that triggers DANGER state (mm). */
    int16_t warning_mm;         /**< Obstacle range that triggers WARNING state (mm). */
    int16_t corridor_deg;       /**< Path-check corridor width (degrees, total). */
    bool    auto_stop;          /**< Issue a swarm STOP when entering DANGER (leader only). */
} avoidance_config_t;

// ============================================================================
// CALLBACKS
// ============================================================================

/**
 * Fired whenever the avoidance state transitions to a new value.
 *
 * @param new_state   The state just entered.
 * @param closest     The obstacle that triggered the transition, or NULL when
 *                    transitioning to CLEAR or DISABLED.
 */
typedef void (*avoidance_state_cb_t)(avoidance_state_t new_state,
                                     const obstacle_detection_t *closest);

/**
 * Fired when the avoidance engine identifies a clear alternate bearing during
 * a DANGER event. The application can use this to steer clear of the obstacle.
 *
 * @param bearing_deg     Suggested heading (degrees, −180 to +180).
 * @param range_clear_mm  The check range used to qualify this bearing as clear.
 */
typedef void (*avoidance_suggest_cb_t)(int16_t bearing_deg, int16_t range_clear_mm);

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * Initialize the avoidance module with default configuration.
 * Must be called before avoidance_start().
 * Safe to call multiple times (idempotent after first call).
 *
 * @return ESP_OK on success.
 */
esp_err_t avoidance_init(void);

/**
 * Start the avoidance check task.
 * obstacle_sense must be initialized and started before calling this.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t avoidance_start(void);

/**
 * Stop the avoidance task. State transitions to DISABLED.
 */
void avoidance_stop(void);

/**
 * Get the current runtime configuration.
 *
 * @param config  Output buffer.
 */
void avoidance_get_config(avoidance_config_t *config);

/**
 * Apply a new configuration. All fields take effect immediately on the next
 * check cycle.
 *
 * @param config  New configuration values.
 */
void avoidance_set_config(const avoidance_config_t *config);

/**
 * Query the current avoidance state.
 *
 * @return Current avoidance_state_t value.
 */
avoidance_state_t avoidance_get_state(void);

/**
 * Convert an avoidance_state_t to a human-readable string.
 */
const char *avoidance_state_to_string(avoidance_state_t state);

/**
 * Check whether a given path is safe according to current obstacle data
 * and avoidance thresholds. Uses the configured corridor_deg width.
 *
 * Convenience wrapper around obstacle_sense_path_clear() using the module's
 * own warning_mm threshold and corridor_deg — useful for pre-flight checks
 * before issuing a GOTO command.
 *
 * @param bearing_deg   Direction to check (−180 to +180).
 * @param range_mm      How far ahead to check (mm).
 * @return              true if the path appears clear.
 */
bool avoidance_is_path_safe(int16_t bearing_deg, int16_t range_mm);

/**
 * Register a callback for state-change events.
 * Only one callback is supported; a second call replaces the first.
 */
void avoidance_register_state_cb(avoidance_state_cb_t cb);

/**
 * Register a callback for alternate-bearing suggestions during DANGER events.
 * Only one callback is supported; a second call replaces the first.
 */
void avoidance_register_suggest_cb(avoidance_suggest_cb_t cb);

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

/**
 * Register avoidance-related console commands with ESP console.
 * Call after node_config_start_console().
 */
void avoidance_register_commands(void);

/**
 * Print a formatted avoidance status summary to stdout.
 */
void avoidance_print_status(void);

#endif // AVOIDANCE_H
