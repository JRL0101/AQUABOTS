/**
 * command_engine.c
 * 
 * Swarm command distribution and execution implementation.
 * Step 5: Leader sends commands, followers execute them.
 */

#include "command_engine.h"
#include "leader_election.h"
#include "node_config.h"
#include "swarm_protocol.h"
#include "swarm_transport.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static uint32_t last_processed_cmd_id = 0;
static const char *TAG = "COMMAND_ENGINE";

// ============================================================================
// INTERNAL STATE
// ============================================================================

static command_state_t current_command;
static command_state_t previous_command;  // For RESUME
static bool running = false;
static uint32_t next_cmd_id = 1;

// Mutex for thread-safe access to command state
static SemaphoreHandle_t cmd_mutex = NULL;

// Callbacks
static command_received_cb_t received_callback = NULL;
static command_status_cb_t status_callback = NULL;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

const char* command_engine_get_type_name(command_type_t type)
{
    switch (type) {
        case CMD_NONE: return "NONE";
        case CMD_GOTO: return "GOTO";
        case CMD_HOLD: return "HOLD";
        case CMD_STOP: return "STOP";
        case CMD_RESUME: return "RESUME";
        case CMD_FORMATION: return "FORMATION";
        default: return "UNKNOWN";
    }
}

const char* command_engine_get_status_name(command_status_t status)
{
    switch (status) {
        case CMD_STATUS_IDLE: return "IDLE";
        case CMD_STATUS_EXECUTING: return "EXECUTING";
        case CMD_STATUS_COMPLETE: return "COMPLETE";
        case CMD_STATUS_FAILED: return "FAILED";
        case CMD_STATUS_HOLDING: return "HOLDING";
        case CMD_STATUS_STOPPED: return "STOPPED";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// COMMAND PROTOCOL
// ============================================================================

static void send_command_packet(const swarm_command_t *cmd)
{
    uint16_t my_node_id = node_config_get_node_id();
    
    // Build swarm packet header
    swarm_header_t header;
    header.swarm_id = 0;
    header.src_id = my_node_id;
    header.dst_id = cmd->target_node;  // 0 = broadcast
    header.group_id = cmd->target_group;
    header.msg_type = MSG_TYPE_COMMAND;
    header.seq = (uint16_t)(cmd->cmd_id & 0xFFFF);  // Truncate to 16-bit (full id in payload)
    header.flags = 0;
    header.payload_len = sizeof(swarm_command_t);
    
    // Combine header + command
    uint8_t packet[sizeof(swarm_header_t) + sizeof(swarm_command_t)];
    memcpy(packet, &header, sizeof(swarm_header_t));
    memcpy(packet + sizeof(swarm_header_t), cmd, sizeof(swarm_command_t));
    
    // Broadcast command
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    swarm_transport_send(broadcast_mac, packet, sizeof(packet));
    
    ESP_LOGI(TAG, "Command sent: type=%s, target=%u, id=%lu",
             command_engine_get_type_name(cmd->cmd_type),
             cmd->target_node,
             (unsigned long)cmd->cmd_id);
}

static void process_command(const swarm_command_t *cmd)
{
    uint16_t my_node_id = node_config_get_node_id();
    
    // Check if command is for us
    if (cmd->target_node != 0 && cmd->target_node != my_node_id) {
        ESP_LOGD(TAG, "Command not for us (target=%u, my_id=%u)", 
                 cmd->target_node, my_node_id);
        return;
    }

    // Check for duplicate command
    if (cmd->cmd_id <= last_processed_cmd_id && cmd->cmd_id != 0) {
        ESP_LOGD(TAG, "Duplicate command ignored: id=%lu", (unsigned long)cmd->cmd_id);
        return;
    }
    last_processed_cmd_id = cmd->cmd_id;
    ESP_LOGI(TAG, "Command received: type=%s, id=%lu, params=(%ld, %ld, %ld)",
             command_engine_get_type_name(cmd->cmd_type),
             (unsigned long)cmd->cmd_id,
             (long)cmd->param1, (long)cmd->param2, (long)cmd->param3);
    
    // Notify callback
    bool accepted = true;
    if (received_callback) {
        accepted = received_callback(cmd);
    }
    
    if (!accepted) {
        ESP_LOGW(TAG, "Command rejected by application");
        return;
    }
    
    // Execute command
    command_engine_execute(cmd);
}

// ============================================================================
// TRANSPORT RECEIVE HANDLER
// ============================================================================

static void command_recv_handler(const uint8_t *src_mac, const uint8_t *data, 
                                 int len, int8_t rssi)
{
    // Parse swarm header
    if (len < sizeof(swarm_header_t)) {
        return;
    }
    
    swarm_header_t header;
    memcpy(&header, data, sizeof(swarm_header_t));
    
    // Only process COMMAND messages
    if (header.msg_type != MSG_TYPE_COMMAND) {
        return;
    }
    
    // Extract command
    if (len < sizeof(swarm_header_t) + sizeof(swarm_command_t)) {
        ESP_LOGW(TAG, "Command message too short");
        return;
    }
    
    swarm_command_t cmd;
    memcpy(&cmd, data + sizeof(swarm_header_t), sizeof(swarm_command_t));
    
    // Process command
    process_command(&cmd);
}

// ============================================================================
// COMMAND EXECUTION
// ============================================================================

esp_err_t command_engine_execute(const swarm_command_t *command)
{
    ESP_LOGI(TAG, "Executing command: %s", command_engine_get_type_name(command->cmd_type));

    xSemaphoreTake(cmd_mutex, portMAX_DELAY);

    // Save previous command for RESUME
    if (current_command.active && command->cmd_type != CMD_RESUME) {
        memcpy(&previous_command, &current_command, sizeof(command_state_t));
    }

    // Handle RESUME specially
    if (command->cmd_type == CMD_RESUME) {
        if (previous_command.active) {
            ESP_LOGI(TAG, "Resuming previous command: %s",
                     command_engine_get_type_name(previous_command.command.cmd_type));
            memcpy(&current_command, &previous_command, sizeof(command_state_t));
            current_command.start_time_ms = esp_log_timestamp();
            current_command.status = CMD_STATUS_EXECUTING;
        } else {
            xSemaphoreGive(cmd_mutex);
            ESP_LOGW(TAG, "No previous command to resume");
            return ESP_ERR_INVALID_STATE;
        }
    } else {
        // Set up new command
        memcpy(&current_command.command, command, sizeof(swarm_command_t));
        current_command.active = true;
        current_command.start_time_ms = esp_log_timestamp();
        current_command.completion_time_ms = 0;

        // Set initial status based on command type
        switch (command->cmd_type) {
            case CMD_STOP:
                current_command.status = CMD_STATUS_STOPPED;
                ESP_LOGI(TAG, "STOP command - all motors off");
                break;

            case CMD_HOLD:
                current_command.status = CMD_STATUS_HOLDING;
                ESP_LOGI(TAG, "HOLD command - maintaining position");
                break;

            case CMD_GOTO:
                current_command.status = CMD_STATUS_EXECUTING;
                ESP_LOGI(TAG, "GOTO command - target: (%ld, %ld), heading: %ld",
                         (long)command->param1, (long)command->param2, (long)command->param3);
                // In a real system, this would start motor control
                // For demo, we'll just simulate execution
                break;

            default:
                current_command.status = CMD_STATUS_EXECUTING;
                break;
        }
    }

    // Copy state for callback (avoid holding mutex during callback)
    swarm_command_t cmd_copy = current_command.command;
    command_status_t status_copy = current_command.status;

    xSemaphoreGive(cmd_mutex);

    // Notify status callback outside mutex
    if (status_callback) {
        status_callback(&cmd_copy, status_copy);
    }

    return ESP_OK;
}

void command_engine_update_status(command_status_t status)
{
    xSemaphoreTake(cmd_mutex, portMAX_DELAY);

    if (!current_command.active) {
        xSemaphoreGive(cmd_mutex);
        return;
    }

    command_status_t old_status = current_command.status;
    current_command.status = status;

    if (status == CMD_STATUS_COMPLETE || status == CMD_STATUS_FAILED) {
        current_command.completion_time_ms = esp_log_timestamp();
        uint32_t duration = current_command.completion_time_ms - current_command.start_time_ms;

        ESP_LOGI(TAG, "Command %s: %s (duration: %lu ms)",
                 command_engine_get_type_name(current_command.command.cmd_type),
                 command_engine_get_status_name(status),
                 (unsigned long)duration);
    }

    // Copy for callback
    swarm_command_t cmd_copy = current_command.command;
    bool should_notify = (old_status != status && status_callback != NULL);

    xSemaphoreGive(cmd_mutex);

    // Notify status callback outside mutex
    if (should_notify) {
        status_callback(&cmd_copy, status);
    }
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t command_engine_init(void)
{
    ESP_LOGI(TAG, "Initializing command engine...");

    // Create mutex for thread safety
    if (cmd_mutex == NULL) {
        cmd_mutex = xSemaphoreCreateMutex();
        if (cmd_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create command mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    memset(&current_command, 0, sizeof(command_state_t));
    memset(&previous_command, 0, sizeof(command_state_t));
    next_cmd_id = 1;
    last_processed_cmd_id = 0;

    ESP_LOGI(TAG, "Command engine initialized");
    return ESP_OK;
}

esp_err_t command_engine_start(void)
{
    if (running) {
        ESP_LOGW(TAG, "Command engine already running");
        return ESP_OK;
    }
    
    uint16_t my_node_id = node_config_get_node_id();
    if (my_node_id == NODE_ID_INVALID) {
        ESP_LOGW(TAG, "Cannot start command engine - node not provisioned");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting command engine...");
    
    // Register receive handler
    swarm_transport_register_recv_cb(command_recv_handler);
    
    running = true;
    
    ESP_LOGI(TAG, "Command engine started");
    return ESP_OK;
}

void command_engine_stop(void)
{
    if (!running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping command engine...");
    running = false;
    
    ESP_LOGI(TAG, "Command engine stopped");
}

esp_err_t command_engine_send_command(command_type_t cmd_type,
                                      uint16_t target_node,
                                      int32_t param1,
                                      int32_t param2,
                                      int32_t param3)
{
    // Only leader can send commands
    if (!leader_election_is_leader()) {
        ESP_LOGW(TAG, "Cannot send command - not leader");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Build command
    swarm_command_t cmd;
    cmd.cmd_type = cmd_type;
    cmd.target_group = 0;  // Default to all groups
    cmd.target_node = target_node;
    cmd.cmd_id = next_cmd_id++;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.timeout_ms = 0;  // No timeout by default
    
    ESP_LOGI(TAG, "Sending command: %s (target=%u)", 
             command_engine_get_type_name(cmd_type), target_node);
    
    // Send to swarm
    send_command_packet(&cmd);
    
    // If target is 0 (broadcast) or includes us, execute locally too
    uint16_t my_node_id = node_config_get_node_id();
    if (target_node == 0 || target_node == my_node_id) {
        command_engine_execute(&cmd);
    }
    
    return ESP_OK;
}

bool command_engine_get_state(command_state_t *state)
{
    xSemaphoreTake(cmd_mutex, portMAX_DELAY);

    if (!current_command.active) {
        xSemaphoreGive(cmd_mutex);
        return false;
    }

    memcpy(state, &current_command, sizeof(command_state_t));

    xSemaphoreGive(cmd_mutex);
    return true;
}

command_status_t command_engine_get_status(void)
{
    xSemaphoreTake(cmd_mutex, portMAX_DELAY);
    command_status_t status = current_command.status;
    xSemaphoreGive(cmd_mutex);
    return status;
}

void command_engine_register_received_cb(command_received_cb_t cb)
{
    received_callback = cb;
}

void command_engine_register_status_cb(command_status_cb_t cb)
{
    status_callback = cb;
}
