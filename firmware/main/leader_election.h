/**
 * leader_election.h
 * 
 * Leader election and TDMA beacon scheduling.
 * Highest node_id wins; the elected leader broadcasts the TDMA slot schedule.
 */

#ifndef LEADER_ELECTION_H
#define LEADER_ELECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define LEADER_ELECTION_INTERVAL_MS  5000   // Re-evaluate leader every 5s
#define TDMA_SLOT_DURATION_MS        50     // Each time slot is 50ms
#define TDMA_BEACON_INTERVAL_MS      1000   // Beacon every 1 second
#define TDMA_MAX_SLOTS               50     // Max nodes in schedule

// ============================================================================
// LEADER STATE
// ============================================================================

typedef enum {
    LEADER_STATE_FOLLOWER,      // Following another leader
    LEADER_STATE_CANDIDATE,     // Evaluating to become leader
    LEADER_STATE_LEADER         // Active leader
} leader_state_t;

// ============================================================================
// TDMA SLOT ASSIGNMENT
// ============================================================================

typedef struct {
    uint16_t node_id;           // Node assigned to this slot
    uint8_t slot_index;         // Slot number (0-49)
    bool active;                // Is this slot assigned?
} tdma_slot_t;

// ============================================================================
// CALLBACKS
// ============================================================================

/**
 * Callback when leader changes
 * @param new_leader_id Node ID of new leader (0 if no leader)
 * @param is_self True if this node is now the leader
 */
typedef void (*leader_changed_cb_t)(uint16_t new_leader_id, bool is_self);

/**
 * Callback when TDMA schedule is updated
 * @param schedule Array of slot assignments
 * @param num_slots Number of slots in schedule
 */
typedef void (*schedule_updated_cb_t)(const tdma_slot_t *schedule, int num_slots);

// ============================================================================
// API FUNCTIONS
// ============================================================================

/**
 * Initialize leader election system
 * @return ESP_OK on success
 */
esp_err_t leader_election_init(void);

/**
 * Start leader election protocol
 * @return ESP_OK on success
 */
esp_err_t leader_election_start(void);

/**
 * Stop leader election protocol
 */
void leader_election_stop(void);

/**
 * Get current leader ID
 * @return Node ID of current leader, or 0 if no leader
 */
uint16_t leader_election_get_leader(void);

/**
 * Check if this node is the leader
 * @return true if this node is the leader
 */
bool leader_election_is_leader(void);

/**
 * Get current leader state
 * @return Current leader state
 */
leader_state_t leader_election_get_state(void);

/**
 * Register callback for leader change events
 * @param cb Callback function
 */
void leader_election_register_leader_changed_cb(leader_changed_cb_t cb);

/**
 * Register callback for schedule update events
 * @param cb Callback function
 */
void leader_election_register_schedule_updated_cb(schedule_updated_cb_t cb);

/**
 * Get TDMA schedule
 * @param schedule Buffer to fill with schedule
 * @param max_slots Maximum slots to return
 * @return Number of slots in schedule
 */
int leader_election_get_schedule(tdma_slot_t *schedule, int max_slots);

/**
 * Get my assigned TDMA slot
 * @return Slot index, or -1 if not assigned
 */
int leader_election_get_my_slot(void);

/**
 * Get time until my next TDMA slot (in ms)
 * @return Milliseconds until slot, or -1 if not assigned
 */
int leader_election_get_time_to_slot(void);

#endif // LEADER_ELECTION_H
