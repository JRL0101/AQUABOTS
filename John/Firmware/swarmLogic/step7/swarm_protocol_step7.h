/**
 * swarm_protocol.h - ADDITIONS FOR STEP 7
 * 
 * Add this to your existing swarm_protocol.h file.
 * Add MSG_TYPE_OBSTACLE to your message type enum.
 */

// ============================================================================
// ADD TO MESSAGE TYPE ENUM
// ============================================================================

/*
 * In your msg_type enum, add:
 *
 *   MSG_TYPE_OBSTACLE = 0x07,   // Obstacle scan broadcast
 *
 * Full enum should now look like:
 *
 * typedef enum {
 *     MSG_TYPE_HELLO      = 0x01,
 *     MSG_TYPE_BEACON     = 0x02,
 *     MSG_TYPE_COMMAND    = 0x03,
 *     MSG_TYPE_ACK        = 0x04,
 *     MSG_TYPE_PING       = 0x05,
 *     MSG_TYPE_FORMATION  = 0x06,   // Added in Step 6
 *     MSG_TYPE_OBSTACLE   = 0x07,   // <-- ADD THIS for Step 7
 * } swarm_msg_type_t;
 */

// ============================================================================
// OBSTACLE MESSAGE TYPE VALUE
// ============================================================================

#ifndef MSG_TYPE_OBSTACLE
#define MSG_TYPE_OBSTACLE  0x07
#endif
