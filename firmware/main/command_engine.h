/**
 * command_engine.h
 * 
 * Swarm command distribution and execution.
 * Leader distributes GOTO, HOLD, STOP, and RESUME commands to all followers.
 */

#ifndef COMMAND_ENGINE_H
#define COMMAND_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// COMMAND TYPES
// ============================================================================

typedef enum {
    CMD_NONE = 0,
    CMD_GOTO,           // Go to waypoint (x, y, heading)
    CMD_HOLD,           // Hold current position
    CMD_STOP,           // Stop all motors
    CMD_RESUME,         // Resume last command
    CMD_FORMATION,      // Move to formation position (future)
} command_type_t;

// ============================================================================
// COMMAND STATUS
// ============================================================================

typedef enum {
    CMD_STATUS_IDLE,        // No active command
    CMD_STATUS_EXECUTING,   // Currently executing
    CMD_STATUS_COMPLETE,    // Command completed successfully
    CMD_STATUS_FAILED,      // Command failed
    CMD_STATUS_HOLDING,     // Holding position
    CMD_STATUS_STOPPED,     // Stopped (motors off)
} command_status_t;

// ============================================================================
// COMMAND STRUCTURE
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t cmd_type;       // command_type_t
    uint8_t target_group;   // 0 = all, or specific group ID
    uint16_t target_node;   // 0 = all, or specific node_id
    uint32_t cmd_id;        // Unique command ID
    int32_t param1;         // Command parameter 1 (e.g., x position)
    int32_t param2;         // Command parameter 2 (e.g., y position)
    int32_t param3;         // Command parameter 3 (e.g., heading)
    uint32_t timeout_ms;    // Command timeout (0 = no timeout)
} swarm_command_t;

// ============================================================================
// COMMAND EXECUTION STATE
// ============================================================================

typedef struct {
    swarm_command_t command;        // Current command
    command_status_t status;        // Execution status
    uint32_t start_time_ms;         // When command started
    uint32_t completion_time_ms;    // When command completed
    bool active;                    // Is this command active?
} command_state_t;

// ============================================================================
// CALLBACKS
// ============================================================================

/**
 * Callback when a new command is received
 * @param command The command to execute
 * @return true if command accepted, false if rejected
 */
typedef bool (*command_received_cb_t)(const swarm_command_t *command);

/**
 * Callback when command status changes
 * @param command The command
 * @param status New status
 */
typedef void (*command_status_cb_t)(const swarm_command_t *command, command_status_t status);

// ============================================================================
// API FUNCTIONS
// ============================================================================

/**
 * Initialize command engine
 * @return ESP_OK on success
 */
esp_err_t command_engine_init(void);

/**
 * Start command engine
 * @return ESP_OK on success
 */
esp_err_t command_engine_start(void);

/**
 * Stop command engine
 */
void command_engine_stop(void);

/**
 * Send a command to the swarm (leader only)
 * @param cmd_type Command type
 * @param target_node Target node (0 = all)
 * @param param1 Parameter 1
 * @param param2 Parameter 2
 * @param param3 Parameter 3
 * @return ESP_OK on success
 */
esp_err_t command_engine_send_command(command_type_t cmd_type, 
                                      uint16_t target_node,
                                      int32_t param1, 
                                      int32_t param2, 
                                      int32_t param3);

/**
 * Execute a received command
 * @param command Command to execute
 * @return ESP_OK on success
 */
esp_err_t command_engine_execute(const swarm_command_t *command);

/**
 * Update command status
 * @param status New status
 */
void command_engine_update_status(command_status_t status);

/**
 * Get current command state
 * @param state Buffer to fill with current state
 * @return true if active command exists
 */
bool command_engine_get_state(command_state_t *state);

/**
 * Get current command status
 * @return Current status
 */
command_status_t command_engine_get_status(void);

/**
 * Register callback for command received events
 * @param cb Callback function
 */
void command_engine_register_received_cb(command_received_cb_t cb);

/**
 * Register callback for status change events
 * @param cb Callback function
 */
void command_engine_register_status_cb(command_status_cb_t cb);

/**
 * Get command type name as string
 * @param type Command type
 * @return String representation
 */
const char* command_engine_get_type_name(command_type_t type);

/**
 * Get status name as string
 * @param status Command status
 * @return String representation
 */
const char* command_engine_get_status_name(command_status_t status);

#endif // COMMAND_ENGINE_H
