/**
 * membership.c
 * 
 * Swarm membership management implementation.
 * Step 3: HELLO broadcasts, member table, timeout tracking.
 */

#include "membership.h"
#include "swarm_protocol.h"
#include "swarm_transport.h"
#include "node_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MEMBERSHIP";

// ============================================================================
// HELLO MESSAGE FORMAT
// ============================================================================

typedef struct __attribute__((packed)) {
    uint16_t node_id;       // Sender's node ID
    uint8_t capabilities;   // Node capabilities
    uint8_t role;           // Node role
    uint32_t uptime_ms;     // Uptime in milliseconds
} hello_payload_t;

// ============================================================================
// INTERNAL STATE
// ============================================================================

static member_info_t member_table[MEMBERSHIP_MAX_MEMBERS];
static int member_count = 0;
static bool running = false;

// Callbacks
static member_joined_cb_t joined_callback = NULL;
static member_lost_cb_t lost_callback = NULL;

// Task handle
static TaskHandle_t hello_task_handle = NULL;
static TaskHandle_t timeout_task_handle = NULL;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static bool mac_equals(const uint8_t *mac1, const uint8_t *mac2)
{
    return memcmp(mac1, mac2, 6) == 0;
}

static int find_member_by_node_id(uint16_t node_id)
{
    for (int i = 0; i < MEMBERSHIP_MAX_MEMBERS; i++) {
        if (member_table[i].active && member_table[i].node_id == node_id) {
            return i;
        }
    }
    return -1;
}

static int find_member_by_mac(const uint8_t *mac)
{
    for (int i = 0; i < MEMBERSHIP_MAX_MEMBERS; i++) {
        if (member_table[i].active && mac_equals(member_table[i].mac, mac)) {
            return i;
        }
    }
    return -1;
}

static int find_free_slot(void)
{
    for (int i = 0; i < MEMBERSHIP_MAX_MEMBERS; i++) {
        if (!member_table[i].active) {
            return i;
        }
    }
    return -1;
}

// ============================================================================
// MEMBER TABLE MANAGEMENT
// ============================================================================

static void add_member(uint16_t node_id, const uint8_t *mac, int8_t rssi)
{
    int slot = find_free_slot();
    if (slot < 0) {
        ESP_LOGW(TAG, "Member table full, cannot add node_id=%u", node_id);
        return;
    }
    
    member_table[slot].node_id = node_id;
    memcpy(member_table[slot].mac, mac, 6);
    member_table[slot].last_rssi = rssi;
    member_table[slot].last_seen_ms = esp_log_timestamp();
    member_table[slot].capabilities = 0;
    member_table[slot].role = 0;
    member_table[slot].active = true;
    
    if (slot >= member_count) {
        member_count = slot + 1;
    }
    
    ESP_LOGI(TAG, "Member joined: node_id=%u, mac=%02x:%02x:%02x:%02x:%02x:%02x, rssi=%d",
             node_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi);
    
    // Add to transport layer
    swarm_transport_add_peer(mac);
    
    // Notify callback
    if (joined_callback) {
        joined_callback(&member_table[slot]);
    }
}

static void update_member(int slot, int8_t rssi)
{
    member_table[slot].last_rssi = rssi;
    member_table[slot].last_seen_ms = esp_log_timestamp();
}

static void remove_member(int slot)
{
    uint16_t node_id = member_table[slot].node_id;
    
    ESP_LOGW(TAG, "Member lost (timeout): node_id=%u", node_id);
    
    // Notify callback before removing
    if (lost_callback) {
        lost_callback(&member_table[slot]);
    }
    
    member_table[slot].active = false;
}

// ============================================================================
// HELLO PROTOCOL
// ============================================================================

static void send_hello(void)
{
    uint16_t my_node_id = node_config_get_node_id();
    
    if (my_node_id == NODE_ID_INVALID) {
        return; // Not provisioned, don't send HELLO
    }
    
    // Prepare HELLO message
    hello_payload_t hello;
    hello.node_id = my_node_id;
    hello.capabilities = 0; // Future use
    hello.role = 0;         // Future use
    hello.uptime_ms = esp_log_timestamp();
    
    // Build swarm packet header
    swarm_header_t header;
    header.swarm_id = 0;  // Default swarm
    header.src_id = my_node_id;
    header.dst_id = SWARM_BROADCAST_ADDR;
    header.group_id = 0;
    header.msg_type = MSG_TYPE_HELLO;
    header.seq = 0;  // Future use
    header.flags = 0;
    header.payload_len = sizeof(hello_payload_t);
    
    // Combine header + payload
    uint8_t packet[sizeof(swarm_header_t) + sizeof(hello_payload_t)];
    memcpy(packet, &header, sizeof(swarm_header_t));
    memcpy(packet + sizeof(swarm_header_t), &hello, sizeof(hello_payload_t));
    
    // Broadcast to all
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    swarm_transport_send(broadcast_mac, packet, sizeof(packet));
    
    ESP_LOGD(TAG, "HELLO broadcast sent: node_id=%u", my_node_id);
}

static void process_hello(const uint8_t *src_mac, const hello_payload_t *hello, int8_t rssi)
{
    uint16_t my_node_id = node_config_get_node_id();
    
    // Ignore our own HELLO messages
    if (hello->node_id == my_node_id) {
        return;
    }
    
    // Check if we already know this member
    int slot = find_member_by_node_id(hello->node_id);
    
    if (slot >= 0) {
        // Known member - update activity
        update_member(slot, rssi);
        ESP_LOGD(TAG, "HELLO from known member: node_id=%u, rssi=%d", hello->node_id, rssi);
    } else {
        // New member - add to table
        add_member(hello->node_id, src_mac, rssi);
    }
}

// ============================================================================
// TRANSPORT RECEIVE HANDLER
// ============================================================================

static void membership_recv_handler(const uint8_t *src_mac, const uint8_t *data, int len, int8_t rssi)
{
    // Parse swarm header
    if (len < sizeof(swarm_header_t)) {
        return; // Too short
    }
    
    swarm_header_t header;
    memcpy(&header, data, sizeof(swarm_header_t));
    
    // Only process HELLO messages
    if (header.msg_type != MSG_TYPE_HELLO) {
        return;
    }
    
    // Extract payload
    if (len < sizeof(swarm_header_t) + sizeof(hello_payload_t)) {
        ESP_LOGW(TAG, "HELLO message too short");
        return;
    }
    
    hello_payload_t hello;
    memcpy(&hello, data + sizeof(swarm_header_t), sizeof(hello_payload_t));
    
    // Process HELLO
    process_hello(src_mac, &hello, rssi);
}

// ============================================================================
// PERIODIC TASKS
// ============================================================================

static void hello_broadcast_task(void *pvParameter)
{
    ESP_LOGI(TAG, "HELLO broadcast task started (interval=%d ms)", MEMBERSHIP_HELLO_INTERVAL_MS);
    
    while (running) {
        send_hello();
        vTaskDelay(pdMS_TO_TICKS(MEMBERSHIP_HELLO_INTERVAL_MS));
    }
    
    ESP_LOGI(TAG, "HELLO broadcast task stopped");
    vTaskDelete(NULL);
}

static void timeout_check_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Timeout check task started (timeout=%d ms)", MEMBERSHIP_TIMEOUT_MS);
    
    while (running) {
        uint32_t now = esp_log_timestamp();
        
        // Check for timed-out members
        for (int i = 0; i < MEMBERSHIP_MAX_MEMBERS; i++) {
            if (member_table[i].active) {
                uint32_t age = now - member_table[i].last_seen_ms;
                if (age > MEMBERSHIP_TIMEOUT_MS) {
                    remove_member(i);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
    
    ESP_LOGI(TAG, "Timeout check task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t membership_init(void)
{
    ESP_LOGI(TAG, "Initializing membership system...");
    
    // Initialize member table
    memset(member_table, 0, sizeof(member_table));
    member_count = 0;
    
    ESP_LOGI(TAG, "Membership system initialized");
    return ESP_OK;
}

esp_err_t membership_start(void)
{
    if (running) {
        ESP_LOGW(TAG, "Membership already running");
        return ESP_OK;
    }
    
    uint16_t my_node_id = node_config_get_node_id();
    if (my_node_id == NODE_ID_INVALID) {
        ESP_LOGW(TAG, "Cannot start membership - node not provisioned");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting membership protocol...");
    
    // Register receive handler
    swarm_transport_register_recv_cb(membership_recv_handler);
    
    running = true;
    
    // Start HELLO broadcast task
    xTaskCreate(hello_broadcast_task, "hello_bcast", 3072, NULL, 5, &hello_task_handle);
    
    // Start timeout check task
    xTaskCreate(timeout_check_task, "timeout_chk", 4096, NULL, 4, &timeout_task_handle);
    
    ESP_LOGI(TAG, "Membership protocol started");
    return ESP_OK;
}

void membership_stop(void)
{
    if (!running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping membership protocol...");
    running = false;
    
    // Tasks will self-terminate
    hello_task_handle = NULL;
    timeout_task_handle = NULL;
    
    ESP_LOGI(TAG, "Membership protocol stopped");
}

void membership_register_joined_cb(member_joined_cb_t cb)
{
    joined_callback = cb;
}

void membership_register_lost_cb(member_lost_cb_t cb)
{
    lost_callback = cb;
}

void membership_update_peer(uint16_t node_id, const uint8_t *mac, int8_t rssi)
{
    int slot = find_member_by_node_id(node_id);
    
    if (slot >= 0) {
        update_member(slot, rssi);
    } else {
        // New member discovered outside of HELLO (e.g., from other message types)
        add_member(node_id, mac, rssi);
    }
}

const member_info_t* membership_get_member(uint16_t node_id)
{
    int slot = find_member_by_node_id(node_id);
    return (slot >= 0) ? &member_table[slot] : NULL;
}

const member_info_t* membership_get_member_by_mac(const uint8_t *mac)
{
    int slot = find_member_by_mac(mac);
    return (slot >= 0) ? &member_table[slot] : NULL;
}

int membership_get_count(void)
{
    int count = 0;
    for (int i = 0; i < MEMBERSHIP_MAX_MEMBERS; i++) {
        if (member_table[i].active) {
            count++;
        }
    }
    return count;
}

int membership_get_all_members(const member_info_t **members, int max_members)
{
    int count = 0;
    for (int i = 0; i < MEMBERSHIP_MAX_MEMBERS && count < max_members; i++) {
        if (member_table[i].active) {
            members[count++] = &member_table[i];
        }
    }
    return count;
}

bool membership_is_member(uint16_t node_id)
{
    return find_member_by_node_id(node_id) >= 0;
}

void membership_print_table(void)
{
    int count = membership_get_count();
    
    printf("\n=== Member Table (%d members) ===\n", count);
    printf("%-8s %-20s %-8s %-10s\n", "Node ID", "MAC Address", "RSSI", "Last Seen");
    printf("----------------------------------------------------------------\n");
    
    for (int i = 0; i < MEMBERSHIP_MAX_MEMBERS; i++) {
        if (member_table[i].active) {
            uint32_t age = esp_log_timestamp() - member_table[i].last_seen_ms;
            printf("%-8u %02x:%02x:%02x:%02x:%02x:%02x  %-8d %lu ms ago\n",
                   member_table[i].node_id,
                   member_table[i].mac[0], member_table[i].mac[1],
                   member_table[i].mac[2], member_table[i].mac[3],
                   member_table[i].mac[4], member_table[i].mac[5],
                   member_table[i].last_rssi,
                   age);
        }
    }
    printf("\n");
}
