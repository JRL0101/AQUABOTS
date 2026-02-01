/**
 * main.c
 * 
 * ESP32 Swarm Framework - Main Entry Point
 * Step 1: Minimal wiring of modules + demo ping/ack
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

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
// TEMPORARY: Step 1 Demo Configuration
// Remove in Step 2 when we add NVS node_id provisioning
// ============================================================================

#ifndef DEMO_ROLE
#define DEMO_ROLE 0  // 0=responder, 1=initiator
#endif

// Known peer MACs (temporary, will be replaced with discovery in Step 3)
__attribute__((unused)) static const uint8_t MAC_A[6] = {0x6c, 0xc8, 0x40, 0x89, 0x71, 0x64};
__attribute__((unused)) static const uint8_t MAC_B[6] = {0x78, 0x1c, 0x3c, 0xf1, 0x5e, 0x74};

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "ESP32 Swarm Framework - Step 1");
    ESP_LOGI(TAG, "==============================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize transport layer
    ESP_ERROR_CHECK(swarm_transport_init());
    
    // Initialize all swarm modules (stubs for Step 1)
    membership_init();
    leader_election_init();
    scheduler_init();
    command_engine_init();
    formation_init();
    
    // Initialize obstacle/avoidance modules (stubs for Step 1)
    obstacle_sense_init();
    avoidance_init();
    
    ESP_LOGI(TAG, "All modules initialized");
    
    // ========================================================================
    // STEP 1 DEMO MODE: Run ping/ack demo to verify scaffolding works
    // ========================================================================
    
#if DEMO_ROLE == 1
    ESP_LOGI(TAG, "=== DEMO MODE: INITIATOR ===");
    ESP_LOGI(TAG, "Will send PINGs to peer every 1 second");
    swarm_transport_start_demo_pingack(1, MAC_B);
#else
    ESP_LOGI(TAG, "=== DEMO MODE: RESPONDER ===");
    ESP_LOGI(TAG, "Waiting for PINGs and will respond with ACKs");
    swarm_transport_start_demo_pingack(0, NULL);
#endif
    
    ESP_LOGI(TAG, "System running. Demo ping/ack active.");
}
