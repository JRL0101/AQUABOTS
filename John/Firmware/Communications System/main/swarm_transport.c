/**
 * swarm_transport.c
 * 
 * ESP-NOW transport layer implementation.
 * Step 1: Wraps existing ping/ack demo code.
 */

#include "swarm_transport.h"
#include "swarm_protocol.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "TRANSPORT";

// ============================================================================
// INTERNAL STATE
// ============================================================================

static transport_peer_t peer_table[TRANSPORT_MAX_PEERS];
static int peer_count = 0;

// Support multiple receive callbacks
#define MAX_RECV_CALLBACKS 4
static transport_recv_cb_t recv_callbacks[MAX_RECV_CALLBACKS] = {NULL};
static int recv_callback_count = 0;

// ============================================================================
// DEMO MODE STATE (Step 1 only)
// ============================================================================

#define MAX_PENDING_PINGS 32

typedef struct {
    uint8_t peer_mac[ESP_NOW_ETH_ALEN];
    uint32_t counter;
    int64_t send_time_us;
    bool pending;
} ping_record_t;

static ping_record_t ping_records[MAX_PENDING_PINGS];
static uint32_t next_record_index = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static bool mac_equals(const uint8_t *mac1, const uint8_t *mac2)
{
    return memcmp(mac1, mac2, ESP_NOW_ETH_ALEN) == 0;
}

static void mac_to_str(const uint8_t *mac, char *str, size_t len)
{
    snprintf(str, len, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int find_peer_slot(const uint8_t *mac)
{
    for (int i = 0; i < TRANSPORT_MAX_PEERS; i++) {
        if (peer_table[i].active && mac_equals(peer_table[i].mac, mac)) {
            return i;
        }
    }
    return -1;
}

// ============================================================================
// RTT TRACKING (Demo mode)
// ============================================================================

static void record_ping_sent(const uint8_t *dest_mac, uint32_t counter, int64_t send_time_us)
{
    uint32_t index = next_record_index % MAX_PENDING_PINGS;
    memcpy(ping_records[index].peer_mac, dest_mac, ESP_NOW_ETH_ALEN);
    ping_records[index].counter = counter;
    ping_records[index].send_time_us = send_time_us;
    ping_records[index].pending = true;
    next_record_index++;
}

static bool find_ping_record(const uint8_t *src_mac, uint32_t counter, int64_t *send_time_us)
{
    for (int i = 0; i < MAX_PENDING_PINGS; i++) {
        if (ping_records[i].pending && 
            ping_records[i].counter == counter &&
            mac_equals(ping_records[i].peer_mac, src_mac)) {
            *send_time_us = ping_records[i].send_time_us;
            ping_records[i].pending = false;
            return true;
        }
    }
    return false;
}

// ============================================================================
// ESP-NOW CALLBACKS
// ============================================================================

static void espnow_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        char mac_str[18];
        mac_to_str(tx_info->des_addr, mac_str, sizeof(mac_str));
        ESP_LOGW(TAG, "Send failed to %s", mac_str);
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                          const uint8_t *data,
                          int len)
{
    const uint8_t *src_mac = recv_info->src_addr;
    int8_t rssi = recv_info->rx_ctrl->rssi;
    
    // Update peer table
    int slot = find_peer_slot(src_mac);
    if (slot >= 0) {
        peer_table[slot].last_rssi = rssi;
        peer_table[slot].last_seen_ms = esp_log_timestamp();
    }
    
    // Call all registered callbacks
    for (int i = 0; i < recv_callback_count; i++) {
        if (recv_callbacks[i]) {
            recv_callbacks[i](src_mac, data, len, rssi);
        }
    }
}

// ============================================================================
// TRANSPORT API IMPLEMENTATION
// ============================================================================

esp_err_t swarm_transport_init(void)
{
    ESP_LOGI(TAG, "Initializing transport layer...");
    
    // Initialize peer table
    memset(peer_table, 0, sizeof(peer_table));
    memset(ping_records, 0, sizeof(ping_records));
    
    // Initialize WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(TRANSPORT_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    // Print our MAC
    uint8_t mac[ESP_NOW_ETH_ALEN];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char mac_str[18];
    mac_to_str(mac, mac_str, sizeof(mac_str));
    ESP_LOGI(TAG, "My MAC: %s", mac_str);
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Add broadcast peer (required for ESP-NOW broadcast)
    esp_now_peer_info_t broadcast_peer = {
        .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .channel = TRANSPORT_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));
    ESP_LOGI(TAG, "Broadcast peer added");

    ESP_LOGI(TAG, "Transport initialized");
    return ESP_OK;
}

void swarm_transport_register_recv_cb(transport_recv_cb_t cb)
{
    if (recv_callback_count < MAX_RECV_CALLBACKS) {
        recv_callbacks[recv_callback_count++] = cb;
        ESP_LOGI(TAG, "Registered receive callback (%d total)", recv_callback_count);
    } else {
        ESP_LOGE(TAG, "Max receive callbacks reached!");
    }
}

esp_err_t swarm_transport_send(const uint8_t *dest_mac, const uint8_t *data, int len)
{
    return esp_now_send(dest_mac, data, len);
}

esp_err_t swarm_transport_add_peer(const uint8_t *mac)
{
    // Check if already exists
    if (find_peer_slot(mac) >= 0) {
        return ESP_OK;
    }
    
    // Find free slot
    for (int i = 0; i < TRANSPORT_MAX_PEERS; i++) {
        if (!peer_table[i].active) {
            memcpy(peer_table[i].mac, mac, ESP_NOW_ETH_ALEN);
            peer_table[i].active = true;
            peer_table[i].last_rssi = 0;
            peer_table[i].last_seen_ms = esp_log_timestamp();
            
            if (i >= peer_count) {
                peer_count = i + 1;
            }
            
            // Add to ESP-NOW
            esp_now_peer_info_t peer_info = {
                .channel = TRANSPORT_CHANNEL,
                .ifidx = WIFI_IF_STA,
                .encrypt = false,
            };
            memcpy(peer_info.peer_addr, mac, ESP_NOW_ETH_ALEN);
            esp_err_t ret = esp_now_add_peer(&peer_info);
            
            if (ret == ESP_OK) {
                char mac_str[18];
                mac_to_str(mac, mac_str, sizeof(mac_str));
                ESP_LOGI(TAG, "Peer added: %s", mac_str);
            }
            
            return ret;
        }
    }
    
    ESP_LOGW(TAG, "Peer table full");
    return ESP_FAIL;
}

bool swarm_transport_peer_exists(const uint8_t *mac)
{
    return find_peer_slot(mac) >= 0;
}

void swarm_transport_get_my_mac(uint8_t *mac)
{
    esp_wifi_get_mac(WIFI_IF_STA, mac);
}

// ============================================================================
// DEMO MODE - PING/ACK (Step 1 only, will be removed)
// ============================================================================

static void demo_recv_handler(const uint8_t *src_mac, const uint8_t *data, int len, int8_t rssi)
{
    if (len != sizeof(demo_msg_t)) {
        ESP_LOGW(TAG, "Demo: unexpected message size %d", len);
        return;
    }
    
    demo_msg_t msg;
    memcpy(&msg, data, sizeof(msg));
    
    char mac_str[18];
    mac_to_str(src_mac, mac_str, sizeof(mac_str));
    
    if (msg.msg_type == MSG_TYPE_PING) {
        // Received PING - send ACK
        ESP_LOGI(TAG, "DEMO: PING rx: counter=%lu, rssi=%d, from=%s",
                 (unsigned long)msg.counter, rssi, mac_str);
        
        // Ensure sender is peer
        if (!swarm_transport_peer_exists(src_mac)) {
            swarm_transport_add_peer(src_mac);
        }
        
        // Send ACK
        demo_msg_t ack;
        ack.msg_type = MSG_TYPE_ACK;
        ack.counter = msg.counter;
        ack.t_ms = msg.t_ms;
        snprintf(ack.text, sizeof(ack.text), "ack%lu", (unsigned long)msg.counter);
        
        esp_err_t result = swarm_transport_send(src_mac, (uint8_t *)&ack, sizeof(ack));
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "DEMO: ACK tx: counter=%lu", (unsigned long)msg.counter);
        }
        
    } else if (msg.msg_type == MSG_TYPE_ACK) {
        // Received ACK - calculate RTT
        int64_t send_time_us;
        if (find_ping_record(src_mac, msg.counter, &send_time_us)) {
            int64_t now_us = esp_timer_get_time();
            double rtt_ms = (now_us - send_time_us) / 1000.0;
            
            ESP_LOGI(TAG, "DEMO: ACK rx: counter=%lu, RTT=%.2f ms, rssi=%d, from=%s",
                     (unsigned long)msg.counter, rtt_ms, rssi, mac_str);
        }
    }
}

static void demo_ping_task(void *pvParameter)
{
    const uint8_t *peer_mac = (const uint8_t *)pvParameter;
    uint32_t counter = 0;
    
    while (1) {
        demo_msg_t msg;
        msg.msg_type = MSG_TYPE_PING;
        msg.counter = counter;
        msg.t_ms = esp_log_timestamp();
        snprintf(msg.text, sizeof(msg.text), "ping%lu", (unsigned long)counter);
        
        // Record send time
        int64_t send_time_us = esp_timer_get_time();
        record_ping_sent(peer_mac, counter, send_time_us);
        
        char mac_str[18];
        mac_to_str(peer_mac, mac_str, sizeof(mac_str));
        ESP_LOGI(TAG, "DEMO: PING tx: counter=%lu, to=%s", (unsigned long)counter, mac_str);
        
        swarm_transport_send(peer_mac, (uint8_t *)&msg, sizeof(msg));
        
        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void swarm_transport_start_demo_pingack(int role, const uint8_t *peer_mac)
{
    ESP_LOGI(TAG, "Starting demo ping/ack mode, role=%d", role);
    
    // Register receive handler
    swarm_transport_register_recv_cb(demo_recv_handler);
    
    if (role == 1) {
        // Initiator - add peer and start ping task
        if (peer_mac) {
            swarm_transport_add_peer(peer_mac);
            xTaskCreate(demo_ping_task, "demo_ping", 4096, (void *)peer_mac, 5, NULL);
            ESP_LOGI(TAG, "DEMO: Initiator mode - sending PINGs");
        }
    } else {
        // Responder - just wait for pings
        ESP_LOGI(TAG, "DEMO: Responder mode - waiting for PINGs");
    }
}