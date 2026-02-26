/**
 * node_config.h
 * 
 * Node identity and configuration management.
 * Provides NVS-based persistent node_id storage and serial provisioning.
 */

#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// NODE IDENTITY
// ============================================================================

#define NODE_ID_INVALID     0xFFFF
#define NODE_ID_MIN         1
#define NODE_ID_MAX         254

// ============================================================================
// API FUNCTIONS
// ============================================================================

/**
 * Initialize node configuration system
 * Loads node_id from NVS or uses default if not provisioned
 * @return ESP_OK on success
 */
esp_err_t node_config_init(void);

/**
 * Get current node ID
 * @return node_id (or NODE_ID_INVALID if not provisioned)
 */
uint16_t node_config_get_node_id(void);

/**
 * Set node ID and persist to NVS
 * @param node_id Node ID to set (1-254)
 * @return ESP_OK on success
 */
esp_err_t node_config_set_node_id(uint16_t node_id);

/**
 * Check if node is provisioned
 * @return true if node has valid node_id
 */
bool node_config_is_provisioned(void);

/**
 * Get node MAC address
 * @param mac Buffer to store MAC (6 bytes)
 */
void node_config_get_mac(uint8_t *mac);

/**
 * Start serial provisioning console
 * Allows setting node_id via serial command
 */
void node_config_start_console(void);

#endif // NODE_CONFIG_H