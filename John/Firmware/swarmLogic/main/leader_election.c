/**
 * leader_election.c
 * 
 * Leader election and TDMA beacon scheduling implementation.
 * Highest node_id wins; the elected leader broadcasts schedule beacons.
 */

#include "leader_election.h"
#include "membership.h"
#include "node_config.h"
#include "swarm_protocol.h"
#include "swarm_transport.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "LEADER_ELECTION";

// ============================================================================
// BEACON MESSAGE FORMAT
// ============================================================================

typedef struct __attribute__((packed)) {
    uint16_t leader_id;         // Current leader's node ID
    uint8_t num_slots;          // Number of slots in schedule
    uint8_t reserved;           // Padding
    uint32_t beacon_seq;        // Beacon sequence number
    // Followed by tdma_slot_t array
} beacon_payload_t;

// ============================================================================
// INTERNAL STATE
// ============================================================================

static leader_state_t current_state = LEADER_STATE_FOLLOWER;
static uint16_t current_leader_id = 0;
static uint32_t last_leader_update_ms = 0;
static uint32_t beacon_sequence = 0;
static bool running = false;

// TDMA schedule
static tdma_slot_t tdma_schedule[TDMA_MAX_SLOTS];
static int num_assigned_slots = 0;
static int my_slot_index = -1;

// Callbacks
static leader_changed_cb_t leader_changed_callback = NULL;
static schedule_updated_cb_t schedule_updated_callback = NULL;

// Task handles
static TaskHandle_t election_task_handle = NULL;
static TaskHandle_t beacon_task_handle = NULL;

static void generate_tdma_schedule(void);

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static const char* state_to_string(leader_state_t state)
{
    switch (state) {
        case LEADER_STATE_FOLLOWER: return "FOLLOWER";
        case LEADER_STATE_CANDIDATE: return "CANDIDATE";
        case LEADER_STATE_LEADER: return "LEADER";
        default: return "UNKNOWN";
    }
}

static void set_state(leader_state_t new_state)
{
    if (current_state != new_state) {
        ESP_LOGI(TAG, "State change: %s -> %s", 
                 state_to_string(current_state), 
                 state_to_string(new_state));
        current_state = new_state;
    }
}

// ============================================================================
// LEADER ELECTION LOGIC
// ============================================================================

static uint16_t evaluate_leader(void)
{
    uint16_t my_node_id = node_config_get_node_id();
    uint16_t highest_id = my_node_id;
    
    // Get all active members
    const member_info_t *members[MEMBERSHIP_MAX_MEMBERS];
    int count = membership_get_all_members(members, MEMBERSHIP_MAX_MEMBERS);
    
    // Find highest node_id
    for (int i = 0; i < count; i++) {
        if (members[i]->node_id > highest_id) {
            highest_id = members[i]->node_id;
        }
    }
    
    return highest_id;
}

static void update_leader(uint16_t new_leader_id)
{
    uint16_t my_node_id = node_config_get_node_id();
    bool was_leader = (current_state == LEADER_STATE_LEADER);
    bool is_leader = (new_leader_id == my_node_id);
    
    if (current_leader_id != new_leader_id) {
        ESP_LOGI(TAG, "Leader changed: %u -> %u %s", 
                 current_leader_id, new_leader_id,
                 is_leader ? "(I am leader!)" : "");
        
        current_leader_id = new_leader_id;
        last_leader_update_ms = esp_log_timestamp();
        
        // Update state
        if (is_leader) {
            set_state(LEADER_STATE_LEADER);
            
            // Generate TDMA schedule
            generate_tdma_schedule();
        } else {
            set_state(LEADER_STATE_FOLLOWER);
        }
        
        // Notify callback
        if (leader_changed_callback) {
            leader_changed_callback(new_leader_id, is_leader);
        }
    } else if (is_leader && !was_leader) {
        // Became leader without ID change (e.g., first boot)
        set_state(LEADER_STATE_LEADER);
        generate_tdma_schedule();
        
        if (leader_changed_callback) {
            leader_changed_callback(new_leader_id, is_leader);
        }
    }
}

// ============================================================================
// TDMA SCHEDULE GENERATION
// ============================================================================

static void generate_tdma_schedule(void)
{
    ESP_LOGI(TAG, "Generating TDMA schedule...");
    
    uint16_t my_node_id = node_config_get_node_id();
    
    // Clear schedule
    memset(tdma_schedule, 0, sizeof(tdma_schedule));
    num_assigned_slots = 0;
    my_slot_index = -1;
    
    // Get all active members
    const member_info_t *members[MEMBERSHIP_MAX_MEMBERS];
    int count = membership_get_all_members(members, MEMBERSHIP_MAX_MEMBERS);
    
    // Assign slot 0 to myself (leader)
    tdma_schedule[0].node_id = my_node_id;
    tdma_schedule[0].slot_index = 0;
    tdma_schedule[0].active = true;
    my_slot_index = 0;
    num_assigned_slots = 1;
    
    ESP_LOGI(TAG, "  Slot 0: Node %u (self)", my_node_id);
    
    // Assign slots to other members (sorted by node_id)
    // Simple algorithm: assign in order of node_id
    for (int i = 0; i < count && num_assigned_slots < TDMA_MAX_SLOTS; i++) {
        uint16_t node_id = members[i]->node_id;
        
        // Skip self (already assigned)
        if (node_id == my_node_id) {
            continue;
        }
        
        tdma_schedule[num_assigned_slots].node_id = node_id;
        tdma_schedule[num_assigned_slots].slot_index = num_assigned_slots;
        tdma_schedule[num_assigned_slots].active = true;
        
        ESP_LOGI(TAG, "  Slot %d: Node %u", num_assigned_slots, node_id);
        
        num_assigned_slots++;
    }
    
    ESP_LOGI(TAG, "TDMA schedule generated: %d slots", num_assigned_slots);
    
    // Notify callback
    if (schedule_updated_callback) {
        schedule_updated_callback(tdma_schedule, num_assigned_slots);
    }
}

// ============================================================================
// BEACON PROTOCOL
// ============================================================================

static void send_beacon(void)
{
    if (current_state != LEADER_STATE_LEADER) {
        return; // Only leader sends beacons
    }
    
    uint16_t my_node_id = node_config_get_node_id();
    
    // Prepare beacon payload
    int payload_size = sizeof(beacon_payload_t) + (num_assigned_slots * sizeof(tdma_slot_t));
    uint8_t payload[payload_size];
    
    beacon_payload_t *beacon = (beacon_payload_t *)payload;
    beacon->leader_id = my_node_id;
    beacon->num_slots = num_assigned_slots;
    beacon->reserved = 0;
    beacon->beacon_seq = beacon_sequence++;
    
    // Copy schedule
    memcpy(payload + sizeof(beacon_payload_t), tdma_schedule, 
           num_assigned_slots * sizeof(tdma_slot_t));
    
    // Build swarm packet header
    swarm_header_t header;
    header.swarm_id = 0;
    header.src_id = my_node_id;
    header.dst_id = SWARM_BROADCAST_ADDR;
    header.group_id = 0;
    header.msg_type = MSG_TYPE_BEACON;
    header.seq = beacon_sequence;
    header.flags = 0;
    header.payload_len = payload_size;
    
    // Combine header + payload
    uint8_t packet[sizeof(swarm_header_t) + payload_size];
    memcpy(packet, &header, sizeof(swarm_header_t));
    memcpy(packet + sizeof(swarm_header_t), payload, payload_size);
    
    // Broadcast beacon
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    swarm_transport_send(broadcast_mac, packet, sizeof(packet));
    
    ESP_LOGD(TAG, "BEACON sent: seq=%lu, slots=%d", beacon_sequence, num_assigned_slots);
}

static void process_beacon(const uint8_t *src_mac, const uint8_t *data, int len)
{
    uint16_t my_node_id = node_config_get_node_id();
    
    // Parse beacon payload
    if (len < sizeof(beacon_payload_t)) {
        ESP_LOGW(TAG, "Beacon too short");
        return;
    }
    
    beacon_payload_t beacon;
    memcpy(&beacon, data, sizeof(beacon_payload_t));
    
    // Ignore beacons from nodes with lower ID (they shouldn't be leader)
    if (beacon.leader_id < my_node_id) {
        ESP_LOGD(TAG, "Ignoring beacon from lower-priority node %u", beacon.leader_id);
        return;
    }
    
    // Update leader if different
    if (current_leader_id != beacon.leader_id) {
        ESP_LOGI(TAG, "Received beacon from new leader: %u", beacon.leader_id);
        update_leader(beacon.leader_id);
    }
    
    last_leader_update_ms = esp_log_timestamp();
    
    // Extract schedule
    int expected_len = sizeof(beacon_payload_t) + (beacon.num_slots * sizeof(tdma_slot_t));
    if (len < expected_len) {
        ESP_LOGW(TAG, "Beacon schedule incomplete");
        return;
    }

    // Check if schedule actually changed before updating
    const uint8_t *new_schedule_data = data + sizeof(beacon_payload_t);
    int new_schedule_size = beacon.num_slots * sizeof(tdma_slot_t);

    bool schedule_changed = (num_assigned_slots != beacon.num_slots) ||
                            (memcmp(tdma_schedule, new_schedule_data, new_schedule_size) != 0);

    if (!schedule_changed) {
        return; // No change, skip update
    }

    // Update schedule
    memset(tdma_schedule, 0, sizeof(tdma_schedule));
    num_assigned_slots = beacon.num_slots;
    memcpy(tdma_schedule, new_schedule_data, new_schedule_size);

    // Find my slot
    my_slot_index = -1;
    for (int i = 0; i < num_assigned_slots; i++) {
        if (tdma_schedule[i].node_id == my_node_id) {
            my_slot_index = i;
            break;
        }
    }

    ESP_LOGI(TAG, "Schedule changed: %d slots, my_slot=%d", num_assigned_slots, my_slot_index);

    // Notify callback
    if (schedule_updated_callback) {
        schedule_updated_callback(tdma_schedule, num_assigned_slots);
    }
}

// ============================================================================
// TRANSPORT RECEIVE HANDLER
// ============================================================================

static void leader_election_recv_handler(const uint8_t *src_mac, const uint8_t *data, 
                                         int len, int8_t rssi)
{
    // Parse swarm header
    if (len < sizeof(swarm_header_t)) {
        return;
    }
    
    swarm_header_t header;
    memcpy(&header, data, sizeof(swarm_header_t));
    
    // Only process BEACON messages
    if (header.msg_type != MSG_TYPE_BEACON) {
        return;
    }
    
    // Extract payload
    int payload_len = len - sizeof(swarm_header_t);
    const uint8_t *payload = data + sizeof(swarm_header_t);
    
    process_beacon(src_mac, payload, payload_len);
}

// ============================================================================
// PERIODIC TASKS
// ============================================================================

static void election_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Leader election task started");

    int last_member_count = 0;

    while (running) {
        uint16_t elected_leader = evaluate_leader();

        // Check if leader has timed out (no beacon for 2x interval)
        // Only check if we're NOT the leader (leaders don't receive their own beacons)
        if (current_leader_id != 0 && current_state != LEADER_STATE_LEADER) {
            uint32_t age = esp_log_timestamp() - last_leader_update_ms;
            if (age > (TDMA_BEACON_INTERVAL_MS * 2)) {
                ESP_LOGW(TAG, "Leader timeout (no beacon for %lu ms)", age);
                current_leader_id = 0;
            }
        }

        // Update leader
        update_leader(elected_leader);

        // If we're the leader, check if membership changed and regenerate schedule
        if (current_state == LEADER_STATE_LEADER) {
            int current_member_count = membership_get_count();
            if (current_member_count != last_member_count) {
                ESP_LOGI(TAG, "Membership changed (%d -> %d), regenerating schedule",
                         last_member_count, current_member_count);
                generate_tdma_schedule();
                last_member_count = current_member_count;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LEADER_ELECTION_INTERVAL_MS));
    }
    
    ESP_LOGI(TAG, "Leader election task stopped");
    vTaskDelete(NULL);
}

static void beacon_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Beacon task started");
    
    while (running) {
        if (current_state == LEADER_STATE_LEADER) {
            send_beacon();
        }
        
        vTaskDelay(pdMS_TO_TICKS(TDMA_BEACON_INTERVAL_MS));
    }
    
    ESP_LOGI(TAG, "Beacon task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t leader_election_init(void)
{
    ESP_LOGI(TAG, "Initializing leader election system...");
    
    current_state = LEADER_STATE_FOLLOWER;
    current_leader_id = 0;
    beacon_sequence = 0;
    num_assigned_slots = 0;
    my_slot_index = -1;
    
    memset(tdma_schedule, 0, sizeof(tdma_schedule));
    
    ESP_LOGI(TAG, "Leader election system initialized");
    return ESP_OK;
}

esp_err_t leader_election_start(void)
{
    if (running) {
        ESP_LOGW(TAG, "Leader election already running");
        return ESP_OK;
    }
    
    uint16_t my_node_id = node_config_get_node_id();
    if (my_node_id == NODE_ID_INVALID) {
        ESP_LOGW(TAG, "Cannot start leader election - node not provisioned");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting leader election protocol...");
    
    // Register receive handler
    swarm_transport_register_recv_cb(leader_election_recv_handler);
    
    running = true;
    
    // Start election task
    xTaskCreate(election_task, "leader_elect", 3072, NULL, 5, &election_task_handle);
    
    // Start beacon task
    xTaskCreate(beacon_task, "beacon", 3072, NULL, 5, &beacon_task_handle);
    
    ESP_LOGI(TAG, "Leader election protocol started");
    return ESP_OK;
}

void leader_election_stop(void)
{
    if (!running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping leader election protocol...");
    running = false;
    
    election_task_handle = NULL;
    beacon_task_handle = NULL;
    
    ESP_LOGI(TAG, "Leader election protocol stopped");
}

uint16_t leader_election_get_leader(void)
{
    return current_leader_id;
}

bool leader_election_is_leader(void)
{
    return (current_state == LEADER_STATE_LEADER);
}

leader_state_t leader_election_get_state(void)
{
    return current_state;
}

void leader_election_register_leader_changed_cb(leader_changed_cb_t cb)
{
    leader_changed_callback = cb;
}

void leader_election_register_schedule_updated_cb(schedule_updated_cb_t cb)
{
    schedule_updated_callback = cb;
}

int leader_election_get_schedule(tdma_slot_t *schedule, int max_slots)
{
    int count = (num_assigned_slots < max_slots) ? num_assigned_slots : max_slots;
    memcpy(schedule, tdma_schedule, count * sizeof(tdma_slot_t));
    return count;
}

int leader_election_get_my_slot(void)
{
    return my_slot_index;
}

int leader_election_get_time_to_slot(void)
{
    if (my_slot_index < 0 || num_assigned_slots == 0) {
        return -1; // Not assigned
    }
    
    // Calculate time to next slot
    // This is simplified - a real implementation would use precise timing
    uint32_t now_ms = esp_log_timestamp() % (num_assigned_slots * TDMA_SLOT_DURATION_MS);
    uint32_t my_slot_ms = my_slot_index * TDMA_SLOT_DURATION_MS;
    
    if (now_ms < my_slot_ms) {
        return my_slot_ms - now_ms;
    } else {
        // Slot already passed, wait for next cycle
        return (num_assigned_slots * TDMA_SLOT_DURATION_MS) - now_ms + my_slot_ms;
    }
}
