#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

void app_main(void)
{
    ESP_LOGI("APP", "Hello. ESP32 booted.");

    while (1) {
        ESP_LOGI("APP", "Alive. Free heap = %lu", (unsigned long)esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
