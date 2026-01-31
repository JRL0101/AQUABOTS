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

static const char *TAG = "ESPNOW";

// ============================================================================
// ROLE SELECTION: Change this to build for Board A or Board B
// ============================================================================
// Set to 1 for Board A (sender)COM3 , 0 for Board B (receiver)COM4
#ifndef ROLE_IS_A
#define ROLE_IS_A 0
#endif

// ============================================================================
// MAC ADDRESSES: Update these with your actual board MAC addresses
// ============================================================================
// Board A MAC address
static const uint8_t MAC_A[ESP_NOW_ETH_ALEN] __attribute__((unused)) = {0x6c, 0xc8, 0x40, 0x89, 0x71, 0x64};

// Board B MAC address
static const uint8_t MAC_B[ESP_NOW_ETH_ALEN] __attribute__((unused)) = {0x78, 0x1c, 0x3c, 0xf1, 0x5e, 0x74};

// ============================================================================
// MESSAGE STRUCTURE
// ============================================================================
typedef struct __attribute__((packed)) {
    uint32_t counter;
    char message[16];
} espnow_msg_t;

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
    ESP_LOGI(TAG, "My MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ============================================================================
// ESP-NOW SEND CALLBACK (IDF 5.5.x signature with wifi_tx_info_t)
// ============================================================================
static void espnow_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    const uint8_t *mac_addr = tx_info->des_addr;

    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Send success to %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGW(TAG, "Send failed to %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }
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

    ESP_LOGI(TAG, "Received: counter=%lu, message='%s', rssi=%d",
             (unsigned long)msg.counter, msg.message, recv_info->rx_ctrl->rssi);
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

    // Add peer
    esp_now_peer_info_t peer_info = {
        .channel = 1,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer_info.peer_addr, get_peer_mac(), ESP_NOW_ETH_ALEN);

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "Peer added: %02x:%02x:%02x:%02x:%02x:%02x",
             peer_info.peer_addr[0], peer_info.peer_addr[1],
             peer_info.peer_addr[2], peer_info.peer_addr[3],
             peer_info.peer_addr[4], peer_info.peer_addr[5]);
}

// ============================================================================
// SENDER TASK (Board A only)
// ============================================================================
static void sender_task(void *pvParameter)
{
    uint32_t counter = 0;
    espnow_msg_t msg;

    while (1) {
        msg.counter = counter;
        snprintf(msg.message, sizeof(msg.message), "ping%lu", (unsigned long)counter);

        ESP_LOGI(TAG, "Sending: counter=%lu, message='%s'",
                 (unsigned long)counter, msg.message);

        esp_err_t result = esp_now_send(get_peer_mac(),
                                       (const uint8_t *)&msg,
                                       sizeof(msg));

        if (result != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(result));
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

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
    ESP_LOGI(TAG, "=== BOARD A: SENDER MODE ===");
    ESP_LOGI(TAG, "Will send messages every 1 second");
    xTaskCreate(sender_task, "sender_task", 4096, NULL, 5, NULL);
#else
    ESP_LOGI(TAG, "=== BOARD B: RECEIVER MODE ===");
    ESP_LOGI(TAG, "Waiting for messages...");
    // Board B just waits for incoming messages (handled by callback)
#endif
}
