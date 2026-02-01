/**
 * membership.h
 * 
 * Swarm membership management (HELLO/JOIN, member table).
 * Step 3: Dynamic peer discovery via periodic HELLO broadcasts.
 */

#ifndef MEMBERSHIP_H
#define MEMBERSHIP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define MEMBERSHIP_MAX_MEMBERS      50
#define MEMBERSHIP_HELLO_INTERVAL_MS 2000   // Send HELLO every 2 seconds
#define MEMBERSHIP_TIMEOUT_MS       10000   // Consider member lost after 10s silence

// ============================================================================
// MEMBER INFORMATION
// ============================================================================

typedef struct {
    uint16_t node_id;           // Node identifier
    uint8_t mac[6];             // MAC address
    int8_t last_rssi;           // Last received signal strength
    uint32_t last_seen_ms;      // Timestamp of last message
    uint8_t capabilities;       // Node capabilities (future use)
    uint8_t role;               // Node role (future use)
    bool active;                // Is this slot active?
} member_info_t;

// ============================================================================
// CALLBACKS
// ============================================================================

/**
 * Callback when a new member joins the swarm
 * @param member Pointer to member info
 */
typedef void (*member_joined_cb_t)(const member_info_t *member);

/**
 * Callback when a member times out (lost)
 * @param member Pointer to member info
 */
typedef void (*member_lost_cb_t)(const member_info_t *member);

// ============================================================================
// API FUNCTIONS
// ============================================================================

/**
 * Initialize membership system
 * @return ESP_OK on success
 */
esp_err_t membership_init(void);

/**
 * Start membership protocol (HELLO broadcasts)
 * @return ESP_OK on success
 */
esp_err_t membership_start(void);

/**
 * Stop membership protocol
 */
void membership_stop(void);

/**
 * Register callback for member joined event
 * @param cb Callback function
 */
void membership_register_joined_cb(member_joined_cb_t cb);

/**
 * Register callback for member lost event
 * @param cb Callback function
 */
void membership_register_lost_cb(member_lost_cb_t cb);

/**
 * Update member activity (called when message received from peer)
 * @param node_id Node ID of peer
 * @param mac MAC address of peer
 * @param rssi Signal strength
 */
void membership_update_peer(uint16_t node_id, const uint8_t *mac, int8_t rssi);

/**
 * Get member info by node_id
 * @param node_id Node ID to look up
 * @return Pointer to member info, or NULL if not found
 */
const member_info_t* membership_get_member(uint16_t node_id);

/**
 * Get member info by MAC address
 * @param mac MAC address to look up
 * @return Pointer to member info, or NULL if not found
 */
const member_info_t* membership_get_member_by_mac(const uint8_t *mac);

/**
 * Get total number of active members
 * @return Number of active members
 */
int membership_get_count(void);

/**
 * Get all active members
 * @param members Array to fill with member pointers
 * @param max_members Maximum number of members to return
 * @return Number of members returned
 */
int membership_get_all_members(const member_info_t **members, int max_members);

/**
 * Check if a node is a known member
 * @param node_id Node ID to check
 * @return true if member exists and is active
 */
bool membership_is_member(uint16_t node_id);

/**
 * Print member table to console
 */
void membership_print_table(void);

#endif // MEMBERSHIP_H
