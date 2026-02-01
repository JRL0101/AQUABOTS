/**
 * swarm_transport.h
 * 
 * ESP-NOW transport layer abstraction.
 * Handles send/receive, peer management, and callbacks.
 */

#ifndef SWARM_TRANSPORT_H
#define SWARM_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_now.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define TRANSPORT_MAX_PEERS     20
#define TRANSPORT_CHANNEL       1

// ============================================================================
// PEER MANAGEMENT
// ============================================================================

typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    int8_t last_rssi;
    uint32_t last_seen_ms;
    bool active;
} transport_peer_t;

// ============================================================================
// CALLBACKS
// ============================================================================

/**
 * Callback for received data
 * @param src_mac Source MAC address
 * @param data Received data buffer
 * @param len Data length
 * @param rssi Signal strength
 */
typedef void (*transport_recv_cb_t)(const uint8_t *src_mac, 
                                    const uint8_t *data, 
                                    int len, 
                                    int8_t rssi);

// ============================================================================
// API FUNCTIONS
// ============================================================================

/**
 * Initialize transport layer (WiFi + ESP-NOW)
 * @return ESP_OK on success
 */
esp_err_t swarm_transport_init(void);

/**
 * Register receive callback
 * @param cb Callback function
 */
void swarm_transport_register_recv_cb(transport_recv_cb_t cb);

/**
 * Send data to specific peer
 * @param dest_mac Destination MAC address
 * @param data Data buffer
 * @param len Data length
 * @return ESP_OK on success
 */
esp_err_t swarm_transport_send(const uint8_t *dest_mac, 
                               const uint8_t *data, 
                               int len);

/**
 * Add peer to transport layer
 * @param mac Peer MAC address
 * @return ESP_OK on success
 */
esp_err_t swarm_transport_add_peer(const uint8_t *mac);

/**
 * Check if peer exists
 * @param mac Peer MAC address
 * @return true if peer exists
 */
bool swarm_transport_peer_exists(const uint8_t *mac);

/**
 * Get our own MAC address
 * @param mac Buffer to store MAC (6 bytes)
 */
void swarm_transport_get_my_mac(uint8_t *mac);

// ============================================================================
// DEMO MODE (Step 1 only - will be removed in Step 2)
// ============================================================================

/**
 * Start demo ping/ack task (for testing scaffolding)
 * @param role 0=responder, 1=initiator
 * @param peer_mac Target peer MAC (for initiator)
 */
void swarm_transport_start_demo_pingack(int role, const uint8_t *peer_mac);

#endif // SWARM_TRANSPORT_H