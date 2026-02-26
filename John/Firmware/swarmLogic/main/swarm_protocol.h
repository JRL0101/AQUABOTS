/**
 * swarm_protocol.h
 *
 * Swarm packet format definitions and protocol constants.
 */

#ifndef SWARM_PROTOCOL_H
#define SWARM_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// PROTOCOL CONSTANTS
// ============================================================================

#define SWARM_MAX_PAYLOAD_SIZE  200  // ESP-NOW limit is ~250, leave room for header
#define SWARM_MAX_NODES         50
#define SWARM_BROADCAST_ADDR    0xFFFF

// ============================================================================
// MESSAGE TYPES
// ============================================================================

typedef enum {
    MSG_TYPE_PING      = 0,  // Demo ping
    MSG_TYPE_ACK       = 1,  // Demo ack
    MSG_TYPE_HELLO     = 2,  // Membership discovery
    MSG_TYPE_BEACON    = 3,  // TDMA schedule
    MSG_TYPE_COMMAND   = 4,  // Swarm command
    MSG_TYPE_FORMATION = 5,  // Formation plan
    MSG_TYPE_TELEMETRY = 6,  // Status updates
    MSG_TYPE_OBSTACLE  = 7,  // Obstacle scan broadcast
} msg_type_t;

// ============================================================================
// SWARM PACKET HEADER
// ============================================================================

typedef struct __attribute__((packed)) {
    uint16_t swarm_id;      // Swarm identifier
    uint16_t src_id;        // Source node_id
    uint16_t dst_id;        // Destination (node_id or SWARM_BROADCAST_ADDR)
    uint8_t group_id;       // Group identifier (0 = all)
    uint8_t msg_type;       // Message type (msg_type_t)
    uint16_t seq;           // Sequence number
    uint8_t flags;          // Protocol flags
    uint8_t payload_len;    // Payload length in bytes
    // Payload follows
} swarm_header_t;

// ============================================================================
// DEMO PACKET (ping/ack format)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // 0=PING, 1=ACK
    uint32_t counter;      // Message sequence number
    uint32_t t_ms;         // Timestamp in milliseconds
    char text[16];         // Optional text field
} demo_msg_t;

#endif // SWARM_PROTOCOL_H
