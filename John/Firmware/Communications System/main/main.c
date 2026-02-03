/**
 * main.c
 * 
 * ESP32 Swarm Framework - Main Entry Point
 * Step 4: Leader election + TDMA beacon scheduling
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_console.h"

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
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "ESP32 Swarm Framework - Step 4");
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
    
    // Initialize obstacle/avoidance modules
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
    
    // Start membership discovery
    ESP_LOGI(TAG, "=== STARTING SWARM PROTOCOLS ===");
    ESP_LOGI(TAG, "Membership discovery active");
    ESP_LOGI(TAG, "Leader election active");
    ESP_LOGI(TAG, "TDMA beacon scheduling active");
    
    membership_start();
    leader_election_start();
    
    ESP_LOGI(TAG, "System running.");
    ESP_LOGI(TAG, "Console commands: help, info, members, leader, schedule");
}
