#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_timer.h"

static const char *TAG = "ESPNOW";

// ============================================================================
// ROLE SELECTION: Change this to build for Board A or Board B
// ============================================================================
// Set to 1 for Board A (sender/initiator) COM3, 0 for Board B (responder) COM4/COM6
#ifndef ROLE_IS_A
#define ROLE_IS_A 0
#endif

// ============================================================================
// MAC ADDRESSES: update / add accordingly
// ============================================================================
// Board A MAC address: 6c:c8:40:89:71:64: (Normal ESP32 Dev Kit C)
static const uint8_t MAC_A[ESP_NOW_ETH_ALEN] __attribute__((unused)) = {0x6c, 0xc8, 0x40, 0x89, 0x71, 0x64};

// Board B MAC address: 78:1c:3c:f1:5e:74 (Sparkfun ESP32 Thing C)
static const uint8_t MAC_B[ESP_NOW_ETH_ALEN] __attribute__((unused)) = {0x78, 0x1c, 0x3c, 0xf1, 0x5e, 0x74};

// ============================================================================
// MESSAGE STRUCTURE - Bidirectional Protocol
// ============================================================================
typedef enum {
    MSG_TYPE_PING = 0,
    MSG_TYPE_ACK  = 1
} msg_type_t;

typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // 0=PING, 1=ACK
    uint32_t counter;      // Message sequence number
    uint32_t t_ms;         // Timestamp in milliseconds
    char text[16];         // Optional text field
} espnow_msg_t;

// ============================================================================
// RTT TRACKING (Board A only)
// ============================================================================
#if ROLE_IS_A
#define MAX_PENDING_PINGS 32

typedef struct {
    uint32_t counter;
    int64_t send_time_us;  // Timestamp when PING was sent (in microseconds)
    bool pending;
} ping_record_t;

static ping_record_t ping_records[MAX_PENDING_PINGS];
static uint32_t next_record_index = 0;

// Store a PING send time
static void record_ping_sent(uint32_t counter, int64_t send_time_us)
{
    uint32_t index = next_record_index % MAX_PENDING_PINGS;
    ping_records[index].counter = counter;
    ping_records[index].send_time_us = send_time_us;
    ping_records[index].pending = true;
    next_record_index++;
}

// Find and mark a PING record as complete, return send time
static bool find_ping_record(uint32_t counter, int64_t *send_time_us)
{
    for (int i = 0; i < MAX_PENDING_PINGS; i++) {
        if (ping_records[i].pending && ping_records[i].counter == counter) {
            *send_time_us = ping_records[i].send_time_us;
            ping_records[i].pending = false;
            return true;
        }
    }
    return false;
}
#endif

// ============================================================================
// HELPER: Get peer MAC address based on role
// ============================================================================
static const uint8_t *get_peer_mac(void)
{
#if ROLE_IS_A
    return MAC_B;  // A sends to B
#else
    return MAC_A;  // B sends to A
#endif
}

// ============================================================================
// HELPER: Add peer dynamically if not present
// ============================================================================
static void ensure_peer_added(const uint8_t *mac_addr)
{
    if (!esp_now_is_peer_exist(mac_addr)) {
        esp_now_peer_info_t peer_info = {
            .channel = 1,
            .ifidx = WIFI_IF_STA,
            .encrypt = false,
        };
        memcpy(peer_info.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
        
        esp_err_t ret = esp_now_add_peer(&peer_info);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Peer added dynamically: %02x:%02x:%02x:%02x:%02x:%02x",
                     mac_addr[0], mac_addr[1], mac_addr[2],
                     mac_addr[3], mac_addr[4], mac_addr[5]);
        } else {
            ESP_LOGW(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        }
    }
}

// ============================================================================
// WIFI INITIALIZATION FOR ESP-NOW
// ============================================================================
static void wifi_init_for_espnow(void)
{
    ESP_LOGI(TAG, "Initializing WiFi for ESP-NOW...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set to channel 1 (both boards must be on same channel)
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // Disable power saving for more reliable communication
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Print our MAC address
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    
#if ROLE_IS_A
    ESP_LOGI(TAG, "[A] My MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#else
    ESP_LOGI(TAG, "[B] My MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
}

// ============================================================================
// ESP-NOW SEND CALLBACK (IDF 5.5.x signature with wifi_tx_info_t)
// ============================================================================
static void espnow_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    const uint8_t *mac_addr = tx_info->des_addr;

#if ROLE_IS_A
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "[A] Send success to %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGW(TAG, "[A] Send failed to %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }
#else
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "[B] Send success to %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGW(TAG, "[B] Send failed to %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }
#endif
}

// ============================================================================
// ESP-NOW RECEIVE CALLBACK (IDF 5.x signature)
// ============================================================================
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                          const uint8_t *data,
                          int len)
{
    if (len != sizeof(espnow_msg_t)) {
        ESP_LOGW(TAG, "Received data size mismatch: %d bytes (expected %d)",
                 len, sizeof(espnow_msg_t));
        return;
    }

    espnow_msg_t msg;
    memcpy(&msg, data, sizeof(msg));
    
    const uint8_t *sender_mac = recv_info->src_addr;
    int8_t rssi = recv_info->rx_ctrl->rssi;

#if ROLE_IS_A
    // Board A: Expect ACK messages
    if (msg.msg_type == MSG_TYPE_ACK) {
        // Find the original PING send time and calculate RTT
        int64_t send_time_us;
        if (find_ping_record(msg.counter, &send_time_us)) {
            int64_t now_us = esp_timer_get_time();
            int64_t rtt_us = now_us - send_time_us;
            double rtt_ms = rtt_us / 1000.0;
            
            ESP_LOGI(TAG, "[A] ACK received: counter=%lu, RTT=%.2f ms, RSSI=%d dBm, peer=%02x:%02x:%02x:%02x:%02x:%02x",
                     (unsigned long)msg.counter, rtt_ms, rssi,
                     sender_mac[0], sender_mac[1], sender_mac[2],
                     sender_mac[3], sender_mac[4], sender_mac[5]);
        } else {
            ESP_LOGW(TAG, "[A] ACK received but no matching PING: counter=%lu, RSSI=%d dBm",
                     (unsigned long)msg.counter, rssi);
        }
    } else {
        ESP_LOGW(TAG, "[A] Unexpected msg_type=%d", msg.msg_type);
    }
#else
    // Board B: Expect PING messages, send ACK
    if (msg.msg_type == MSG_TYPE_PING) {
        ESP_LOGI(TAG, "[B] PING received: counter=%lu, t_ms=%lu, text='%s', RSSI=%d dBm, from=%02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned long)msg.counter,
                 (unsigned long)msg.t_ms,
                 msg.text,
                 rssi,
                 sender_mac[0], sender_mac[1], sender_mac[2],
                 sender_mac[3], sender_mac[4], sender_mac[5]);
        
        // Ensure sender is in our peer list
        ensure_peer_added(sender_mac);
        
        // Send ACK back with same counter and timestamp
        espnow_msg_t ack;
        ack.msg_type = MSG_TYPE_ACK;
        ack.counter = msg.counter;
        ack.t_ms = msg.t_ms;  // Echo back original timestamp
        snprintf(ack.text, sizeof(ack.text), "ack%lu", (unsigned long)msg.counter);
        
        esp_err_t result = esp_now_send(sender_mac, (const uint8_t *)&ack, sizeof(ack));
        
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "[B] ACK sent: counter=%lu", (unsigned long)msg.counter);
        } else {
            ESP_LOGE(TAG, "[B] ACK send failed: %s", esp_err_to_name(result));
        }
    } else {
        ESP_LOGW(TAG, "[B] Unexpected msg_type=%d", msg.msg_type);
    }
#endif
}

// ============================================================================
// ESP-NOW INITIALIZATION
// ============================================================================
static void espnow_init(void)
{
    ESP_LOGI(TAG, "Initializing ESP-NOW...");

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Add initial peer
    ensure_peer_added(get_peer_mac());
}

// ============================================================================
// SENDER TASK (Board A only)
// ============================================================================
#if ROLE_IS_A
static void sender_task(void *pvParameter)
{
    uint32_t counter = 0;
    espnow_msg_t msg;

    while (1) {
        msg.msg_type = MSG_TYPE_PING;
        msg.counter = counter;
        msg.t_ms = esp_log_timestamp();  // Current time in ms
        snprintf(msg.text, sizeof(msg.text), "ping%lu", (unsigned long)counter);

        // Record send time for RTT calculation
        int64_t send_time_us = esp_timer_get_time();
        record_ping_sent(counter, send_time_us);

        ESP_LOGI(TAG, "[A] Sending PING: counter=%lu, t_ms=%lu, text='%s'",
                 (unsigned long)counter,
                 (unsigned long)msg.t_ms,
                 msg.text);

        esp_err_t result = esp_now_send(get_peer_mac(),
                                       (const uint8_t *)&msg,
                                       sizeof(msg));

        if (result != ESP_OK) {
            ESP_LOGE(TAG, "[A] esp_now_send failed: %s", esp_err_to_name(result));
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

// ============================================================================
// MAIN ENTRY POINT
// ============================================================================
void app_main(void)
{
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi and ESP-NOW
    wifi_init_for_espnow();
    espnow_init();

#if ROLE_IS_A
    ESP_LOGI(TAG, "=== BOARD A: SENDER/INITIATOR MODE ===");
    ESP_LOGI(TAG, "Will send PING messages every 1 second");
    
    // Initialize ping records
    memset(ping_records, 0, sizeof(ping_records));
    
    xTaskCreate(sender_task, "sender_task", 4096, NULL, 5, NULL);
#else
    ESP_LOGI(TAG, "=== BOARD B: RECEIVER/RESPONDER MODE ===");
    ESP_LOGI(TAG, "Waiting for PING messages and will respond with ACK...");
    // Board B just waits for incoming messages (handled by callback)
#endif
}