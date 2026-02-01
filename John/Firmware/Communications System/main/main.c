/**
 * main.c
 * 
 * ESP32 Swarm Framework - Main Entry Point
 * HELLO/JOIN membership discovery, auto-discovery
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
// CONSOLE COMMANDS 
// ============================================================================

static int cmd_members(int argc, char **argv)
{
    membership_print_table();
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
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "ESP32 Swarm Framework");
    ESP_LOGI(TAG, "==============================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize node configuration (loads node_id from NVS)
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
    
    // Start serial console for provisioning
    node_config_start_console();
    
    // Register additional console commands
    register_console_commands();
    
    // ========================================================================
    //  Start membership discovery if provisioned
    // ========================================================================
    
    if (!node_config_is_provisioned()) {
        ESP_LOGW(TAG, "=== NODE NOT PROVISIONED ===");
        ESP_LOGW(TAG, "Entering console mode only.");
        ESP_LOGW(TAG, "Use 'set_node_id <id>' to provision, then restart.");
        ESP_LOGI(TAG, "Available commands: help, info, get_node_id, set_node_id");
        return;
    }
    
    // Register membership callbacks
    membership_register_joined_cb(on_member_joined);
    membership_register_lost_cb(on_member_lost);
    
    // Start membership discovery
    ESP_LOGI(TAG, "=== STARTING SWARM MEMBERSHIP DISCOVERY ===");
    ESP_LOGI(TAG, "Node will broadcast HELLO every 2 seconds");
    ESP_LOGI(TAG, "Auto-discovering peers...");
    
    membership_start();
    
    ESP_LOGI(TAG, "System running. Membership discovery active.");
    ESP_LOGI(TAG, "Console commands: help, info, members");
    ESP_LOGI(TAG, "Type 'members' to see discovered nodes.");
}
