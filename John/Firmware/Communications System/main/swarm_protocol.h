/**
 * swarm_protocol.h
 * 
 * Swarm packet format definitions and protocol constants.
 * Step 1: Basic definitions for future use. Demo still uses simple ping/ack.
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
// MESSAGE TYPES (will expand in future steps)
// ============================================================================

typedef enum {
    MSG_TYPE_PING = 0,      // Demo ping (Step 1)
    MSG_TYPE_ACK  = 1,      // Demo ack (Step 1)
    MSG_TYPE_HELLO = 2,     // Membership discovery (Step 3)
    MSG_TYPE_BEACON = 3,    // TDMA schedule (Step 4)
    MSG_TYPE_COMMAND = 4,   // Swarm command (Step 5)
    MSG_TYPE_FORMATION = 5, // Formation plan (Step 6)
    MSG_TYPE_TELEMETRY = 6, // Status updates
} msg_type_t;

// ============================================================================
// SWARM PACKET HEADER (for future steps)
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
// DEMO PACKET (Step 1 - keeping current ping/ack format)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // 0=PING, 1=ACK
    uint32_t counter;      // Message sequence number
    uint32_t t_ms;         // Timestamp in milliseconds
    char text[16];         // Optional text field
} demo_msg_t;

#endif // SWARM_PROTOCOL_H