/**
 * main.c
 * 
 * ESP32 Swarm Framework - Main Entry Point
 * Step 2: NVS-based node identity, serial provisioning
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

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
// STEP 2 DEMO: Use node_id to determine behavior
// ============================================================================

// Known peer MACs for demo mode (Step 2 only - will be replaced with discovery in Step 3)
// Map node_id to MAC address
typedef struct {
    uint16_t node_id;
    uint8_t mac[6];
} node_mac_map_t;

static const node_mac_map_t KNOWN_NODES[] = {
    {1, {0x6c, 0xc8, 0x40, 0x89, 0x71, 0x64}},  // Node 1: Board A
    {2, {0x78, 0x1c, 0x3c, 0xf1, 0x5e, 0x74}},  // Node 2: Board B
    // Add more nodes here as needed
};

#define KNOWN_NODES_COUNT (sizeof(KNOWN_NODES) / sizeof(KNOWN_NODES[0]))

// Find MAC address for a given node_id
static const uint8_t *find_mac_by_node_id(uint16_t node_id)
{
    for (int i = 0; i < KNOWN_NODES_COUNT; i++) {
        if (KNOWN_NODES[i].node_id == node_id) {
            return KNOWN_NODES[i].mac;
        }
    }
    return NULL;
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "ESP32 Swarm Framework - Step 2");
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
    
    // Initialize all swarm modules (stubs for Step 2)
    membership_init();
    leader_election_init();
    scheduler_init();
    command_engine_init();
    formation_init();
    
    // Initialize obstacle/avoidance modules (stubs for Step 2)
    obstacle_sense_init();
    avoidance_init();
    
    ESP_LOGI(TAG, "All modules initialized");
    
    // Start serial console for provisioning
    node_config_start_console();
    
    // ========================================================================
    // STEP 2 DEMO MODE: Use node_id to determine behavior
    // ========================================================================
    
    if (!node_config_is_provisioned()) {
        ESP_LOGW(TAG, "=== NODE NOT PROVISIONED ===");
        ESP_LOGW(TAG, "Entering console mode only.");
        ESP_LOGW(TAG, "Use 'set_node_id <id>' to provision, then restart.");
        ESP_LOGI(TAG, "Available commands: help, info, get_node_id, set_node_id");
        // Console is running, just return
        return;
    }
    
    // Provisioned - determine demo role based on node_id
    if (my_node_id == 1) {
        // Node 1: Act as initiator, send to Node 2
        ESP_LOGI(TAG, "=== NODE %u: INITIATOR MODE ===", my_node_id);
        ESP_LOGI(TAG, "Will send PINGs to Node 2 every 1 second");
        
        const uint8_t *peer_mac = find_mac_by_node_id(2);
        if (peer_mac) {
            swarm_transport_start_demo_pingack(1, peer_mac);
        } else {
            ESP_LOGE(TAG, "ERROR: Node 2 MAC not found in KNOWN_NODES");
        }
        
    } else {
        // Other nodes: Act as responders
        ESP_LOGI(TAG, "=== NODE %u: RESPONDER MODE ===", my_node_id);
        ESP_LOGI(TAG, "Waiting for PINGs and will respond with ACKs");
        swarm_transport_start_demo_pingack(0, NULL);
    }
    
    ESP_LOGI(TAG, "System running. Demo ping/ack active.");
    ESP_LOGI(TAG, "Console available - type 'help' for commands.");
}