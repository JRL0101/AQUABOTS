/**
 * main.c
 * 
 * ESP32 Swarm Framework - Main Entry Point
 * Step 7: Obstacle sensor scaffolding with mock data
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_console.h"
#include <stdlib.h>

// Node configuration
#include "node_config.h"

// Swarm modules
#include "swarm_transport.h"
#include "membership.h"
#include "leader_election.h"
#include "scheduler.h"
#include "command_engine.h"
#include "formation.h"
#include "obstacle_sense.h"
#include "avoidance.h"

static const char *TAG = "MAIN";

// ============================================================================
// MEMBERSHIP CALLBACKS
// ============================================================================

static void on_member_joined(const member_info_t *member)
{
    ESP_LOGI(TAG, "*** NEW MEMBER DISCOVERED ***");
    ESP_LOGI(TAG, "    Node ID: %u", member->node_id);
    ESP_LOGI(TAG, "    MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             member->mac[0], member->mac[1], member->mac[2],
             member->mac[3], member->mac[4], member->mac[5]);
    ESP_LOGI(TAG, "    RSSI: %d dBm", member->last_rssi);
}

static void on_member_lost(const member_info_t *member)
{
    ESP_LOGW(TAG, "*** MEMBER LOST (TIMEOUT) ***");
    ESP_LOGW(TAG, "    Node ID: %u", member->node_id);
}

// ============================================================================
// LEADER ELECTION CALLBACKS
// ============================================================================

static void on_leader_changed(uint16_t new_leader_id, bool is_self)
{
    if (is_self) {
        ESP_LOGI(TAG, "*** I AM NOW THE LEADER ***");
        ESP_LOGI(TAG, "    Node ID: %u", new_leader_id);
        ESP_LOGI(TAG, "    Generating TDMA schedule...");
    } else {
        ESP_LOGI(TAG, "*** NEW LEADER ELECTED ***");
        ESP_LOGI(TAG, "    Leader Node ID: %u", new_leader_id);
    }
}

static void on_schedule_updated(const tdma_slot_t *schedule, int num_slots)
{
    ESP_LOGI(TAG, "*** TDMA SCHEDULE UPDATED ***");
    ESP_LOGI(TAG, "    Number of slots: %d", num_slots);
    
    int my_slot = leader_election_get_my_slot();
    if (my_slot >= 0) {
        ESP_LOGI(TAG, "    My slot: %d (at %d ms)", my_slot, my_slot * 50);
    } else {
        ESP_LOGW(TAG, "    My slot: NOT ASSIGNED");
    }
}

// ============================================================================
// COMMAND ENGINE CALLBACKS
// ============================================================================

static bool on_command_received(const swarm_command_t *command)
{
    ESP_LOGI(TAG, "*** COMMAND RECEIVED ***");
    ESP_LOGI(TAG, "    Type: %s", command_engine_get_type_name(command->cmd_type));
    ESP_LOGI(TAG, "    ID: %lu", (unsigned long)command->cmd_id);
    ESP_LOGI(TAG, "    Target: %u", command->target_node);
    
    if (command->cmd_type == CMD_GOTO) {
        ESP_LOGI(TAG, "    Waypoint: (%ld, %ld)", (long)command->param1, (long)command->param2);
        ESP_LOGI(TAG, "    Heading: %ld", (long)command->param3);
    }
    
    return true;  // Accept all commands
}

static void on_command_status(const swarm_command_t *command, command_status_t status)
{
    ESP_LOGI(TAG, "*** COMMAND STATUS CHANGE ***");
    ESP_LOGI(TAG, "    Command: %s", command_engine_get_type_name(command->cmd_type));
    ESP_LOGI(TAG, "    Status: %s", command_engine_get_status_name(status));
}

// ============================================================================
// FORMATION CALLBACKS
// ============================================================================

static void on_formation_changed(const formation_config_t *config)
{
    ESP_LOGI(TAG, "*** FORMATION CHANGED ***");
    ESP_LOGI(TAG, "    Type: %s", formation_type_to_string(config->type));
    ESP_LOGI(TAG, "    Nodes: %d", config->node_count);
    ESP_LOGI(TAG, "    Spacing: %ld mm", (long)config->spacing_mm);
    ESP_LOGI(TAG, "    Center: (%ld, %ld) mm", 
             (long)config->center_x_mm, (long)config->center_y_mm);
}

static void on_position_assigned(const formation_position_t *position, int node_index)
{
    ESP_LOGI(TAG, "*** POSITION ASSIGNED ***");
    ESP_LOGI(TAG, "    Formation index: %d%s", node_index, node_index == 0 ? " (leader)" : "");
    ESP_LOGI(TAG, "    Offset: (%ld, %ld) mm", (long)position->x_mm, (long)position->y_mm);
    ESP_LOGI(TAG, "    Heading offset: %d°", position->heading_deg);
}

// ============================================================================
// OBSTACLE SENSING CALLBACKS
// ============================================================================

static void on_obstacle_scan(const obstacle_scan_t *scan)
{
    if (scan->num_detections > 0) {
        ESP_LOGI(TAG, "*** LOCAL SCAN: %d obstacles detected ***", scan->num_detections);
        for (int i = 0; i < scan->num_detections && i < 3; i++) {  // Show first 3
            ESP_LOGI(TAG, "    [%d] %d mm @ %d°", 
                     i, scan->detections[i].range_mm, scan->detections[i].bearing_deg);
        }
    }
}

static void on_remote_obstacle(uint16_t node_id, const obstacle_scan_t *scan)
{
    if (scan->num_detections > 0) {
        ESP_LOGI(TAG, "*** REMOTE SCAN from node %u: %d obstacles ***", 
                 node_id, scan->num_detections);
    }
}

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

static int cmd_members(int argc, char **argv)
{
    membership_print_table();
    return 0;
}

static int cmd_leader(int argc, char **argv)
{
    uint16_t leader_id = leader_election_get_leader();
    bool is_leader = leader_election_is_leader();
    
    printf("\n=== Leader Information ===\n");
    if (leader_id == 0) {
        printf("Leader: NONE (no leader elected yet)\n");
    } else {
        printf("Leader Node ID: %u\n", leader_id);
        printf("I am leader: %s\n", is_leader ? "YES" : "NO");
    }
    
    leader_state_t state = leader_election_get_state();
    const char *state_str = "UNKNOWN";
    switch (state) {
        case LEADER_STATE_FOLLOWER: state_str = "FOLLOWER"; break;
        case LEADER_STATE_CANDIDATE: state_str = "CANDIDATE"; break;
        case LEADER_STATE_LEADER: state_str = "LEADER"; break;
    }
    printf("State: %s\n", state_str);
    printf("\n");
    
    return 0;
}

static int cmd_schedule(int argc, char **argv)
{
    tdma_slot_t schedule[TDMA_MAX_SLOTS];
    int num_slots = leader_election_get_schedule(schedule, TDMA_MAX_SLOTS);
    
    printf("\n=== TDMA Schedule (%d slots) ===\n", num_slots);
    if (num_slots == 0) {
        printf("No schedule available yet.\n\n");
        return 0;
    }
    
    printf("%-8s %-10s %-15s\n", "Slot", "Node ID", "Time (ms)");
    printf("--------------------------------------------\n");
    
    int my_slot = leader_election_get_my_slot();
    for (int i = 0; i < num_slots; i++) {
        if (schedule[i].active) {
            const char *marker = (i == my_slot) ? " <-- ME" : "";
            printf("%-8d %-10u %-15d%s\n", 
                   i, 
                   schedule[i].node_id, 
                   i * TDMA_SLOT_DURATION_MS,
                   marker);
        }
    }
    printf("\n");
    
    if (my_slot >= 0) {
        int time_to_slot = leader_election_get_time_to_slot();
        printf("My slot: %d\n", my_slot);
        printf("Time to my slot: %d ms\n", time_to_slot);
    } else {
        printf("My slot: NOT ASSIGNED\n");
    }
    printf("\n");
    
    return 0;
}

static int cmd_goto(int argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        printf("Usage: goto <x> <y> [heading]\n");
        printf("  x, y: Target position (in meters)\n");
        printf("  heading: Target heading in degrees (optional, default 0)\n");
        printf("\nExamples:\n");
        printf("  goto 10 20        # Go to (10, 20), heading 0\n");
        printf("  goto 10 20 90     # Go to (10, 20), heading 90°\n");
        return 1;
    }
    
    int32_t x = atoi(argv[1]);
    int32_t y = atoi(argv[2]);
    int32_t heading = (argc >= 4) ? atoi(argv[3]) : 0;
    
    printf("Sending GOTO command: (%ld, %ld), heading %ld°\n", 
           (long)x, (long)y, (long)heading);
    
    esp_err_t err = command_engine_send_command(CMD_GOTO, 0, x, y, heading);
    if (err == ESP_OK) {
        printf("Command sent to swarm\n");
    } else {
        printf("Failed to send command: %s\n", esp_err_to_name(err));
        if (err == ESP_ERR_INVALID_STATE) {
            printf("(Only leader can send commands)\n");
        }
    }
    
    return 0;
}

static int cmd_hold(int argc, char **argv)
{
    printf("Sending HOLD command to swarm\n");
    
    esp_err_t err = command_engine_send_command(CMD_HOLD, 0, 0, 0, 0);
    if (err == ESP_OK) {
        printf("Command sent\n");
    } else {
        printf("Failed to send command: %s\n", esp_err_to_name(err));
        if (err == ESP_ERR_INVALID_STATE) {
            printf("(Only leader can send commands)\n");
        }
    }
    
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    printf("Sending STOP command to swarm\n");
    
    esp_err_t err = command_engine_send_command(CMD_STOP, 0, 0, 0, 0);
    if (err == ESP_OK) {
        printf("Command sent\n");
    } else {
        printf("Failed to send command: %s\n", esp_err_to_name(err));
        if (err == ESP_ERR_INVALID_STATE) {
            printf("(Only leader can send commands)\n");
        }
    }
    
    return 0;
}

static int cmd_resume(int argc, char **argv)
{
    printf("Sending RESUME command to swarm\n");
    
    esp_err_t err = command_engine_send_command(CMD_RESUME, 0, 0, 0, 0);
    if (err == ESP_OK) {
        printf("Command sent\n");
    } else {
        printf("Failed to send command: %s\n", esp_err_to_name(err));
        if (err == ESP_ERR_INVALID_STATE) {
            printf("(Only leader can send commands)\n");
        }
    }
    
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    command_state_t state;
    
    printf("\n=== Command Status ===\n");
    
    if (!command_engine_get_state(&state)) {
        printf("Status: IDLE (no active command)\n\n");
        return 0;
    }
    
    printf("Command: %s\n", command_engine_get_type_name(state.command.cmd_type));
    printf("Status: %s\n", command_engine_get_status_name(state.status));
    printf("Command ID: %lu\n", (unsigned long)state.command.cmd_id);
    
    if (state.command.cmd_type == CMD_GOTO) {
        printf("Target: (%ld, %ld)\n", (long)state.command.param1, (long)state.command.param2);
        printf("Heading: %ld°\n", (long)state.command.param3);
    }
    
    uint32_t now = esp_log_timestamp();
    uint32_t elapsed = now - state.start_time_ms;
    printf("Elapsed time: %lu ms\n", (unsigned long)elapsed);
    
    if (state.completion_time_ms > 0) {
        uint32_t duration = state.completion_time_ms - state.start_time_ms;
        printf("Duration: %lu ms\n", (unsigned long)duration);
    }
    
    printf("\n");
    return 0;
}

static void register_console_commands(void)
{
    const esp_console_cmd_t members_cmd = {
        .command = "members",
        .help = "Show member table",
        .hint = NULL,
        .func = &cmd_members,
    };
    esp_console_cmd_register(&members_cmd);
    
    const esp_console_cmd_t leader_cmd = {
        .command = "leader",
        .help = "Show leader information",
        .hint = NULL,
        .func = &cmd_leader,
    };
    esp_console_cmd_register(&leader_cmd);
    
    const esp_console_cmd_t schedule_cmd = {
        .command = "schedule",
        .help = "Show TDMA schedule",
        .hint = NULL,
        .func = &cmd_schedule,
    };
    esp_console_cmd_register(&schedule_cmd);
    
    const esp_console_cmd_t goto_cmd = {
        .command = "goto",
        .help = "Send GOTO command (leader only)",
        .hint = NULL,
        .func = &cmd_goto,
    };
    esp_console_cmd_register(&goto_cmd);
    
    const esp_console_cmd_t hold_cmd = {
        .command = "hold",
        .help = "Send HOLD command (leader only)",
        .hint = NULL,
        .func = &cmd_hold,
    };
    esp_console_cmd_register(&hold_cmd);
    
    const esp_console_cmd_t stop_cmd = {
        .command = "stop",
        .help = "Send STOP command (leader only)",
        .hint = NULL,
        .func = &cmd_stop,
    };
    esp_console_cmd_register(&stop_cmd);
    
    const esp_console_cmd_t resume_cmd = {
        .command = "resume",
        .help = "Send RESUME command (leader only)",
        .hint = NULL,
        .func = &cmd_resume,
    };
    esp_console_cmd_register(&resume_cmd);
    
    const esp_console_cmd_t status_cmd = {
        .command = "status",
        .help = "Show current command status",
        .hint = NULL,
        .func = &cmd_status,
    };
    esp_console_cmd_register(&status_cmd);
    
    // Register formation console commands
    formation_register_commands();
    
    // Register obstacle sensing console commands
    obstacle_sense_register_commands();
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "ESP32 Swarm Framework - Step 7");
    ESP_LOGI(TAG, "==============================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize node configuration
    node_config_init();
    
    // Get our node identity
    uint16_t my_node_id = node_config_get_node_id();
    uint8_t my_mac[6];
    node_config_get_mac(my_mac);
    
    ESP_LOGI(TAG, "==============================================");
    if (node_config_is_provisioned()) {
        ESP_LOGI(TAG, "Node ID: %u", my_node_id);
    } else {
        ESP_LOGI(TAG, "Node ID: NOT PROVISIONED");
        ESP_LOGI(TAG, "Use serial command: set_node_id <id>");
    }
    ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "==============================================");
    
    // Initialize transport layer
    swarm_transport_init();
    
    // Initialize all swarm modules
    membership_init();
    leader_election_init();
    scheduler_init();
    command_engine_init();
    formation_init();
    obstacle_sense_init();
    avoidance_init();
    
    ESP_LOGI(TAG, "All modules initialized");
    
    // Start serial console
    node_config_start_console();
    
    // Register console commands
    register_console_commands();
    
    if (!node_config_is_provisioned()) {
        ESP_LOGW(TAG, "=== NODE NOT PROVISIONED ===");
        ESP_LOGW(TAG, "Entering console mode only.");
        ESP_LOGW(TAG, "Use 'set_node_id <id>' to provision, then restart.");
        return;
    }
    
    // Register callbacks
    membership_register_joined_cb(on_member_joined);
    membership_register_lost_cb(on_member_lost);
    leader_election_register_leader_changed_cb(on_leader_changed);
    leader_election_register_schedule_updated_cb(on_schedule_updated);
    command_engine_register_received_cb(on_command_received);
    command_engine_register_status_cb(on_command_status);
    formation_register_changed_cb(on_formation_changed);
    formation_register_position_cb(on_position_assigned);
    obstacle_sense_register_scan_cb(on_obstacle_scan);
    obstacle_sense_register_remote_cb(on_remote_obstacle);
    
    // Start swarm protocols
    ESP_LOGI(TAG, "=== STARTING SWARM PROTOCOLS ===");
    ESP_LOGI(TAG, "Membership discovery active");
    ESP_LOGI(TAG, "Leader election active");
    ESP_LOGI(TAG, "TDMA beacon scheduling active");
    ESP_LOGI(TAG, "Command engine active");
    ESP_LOGI(TAG, "Formation control active");
    ESP_LOGI(TAG, "Obstacle sensing active (mock mode)");
    
    membership_start();
    leader_election_start();
    command_engine_start();
    formation_start();
    obstacle_sense_start();
    
    ESP_LOGI(TAG, "System running.");
    ESP_LOGI(TAG, "Console commands: help, info, members, leader, schedule,");
    ESP_LOGI(TAG, "                  goto, hold, stop, resume, status,");
    ESP_LOGI(TAG, "                  formation, spacing, fmove, fstatus,");
    ESP_LOGI(TAG, "                  sensors, detections, mock_add, mock_clear, path_check");
}
