/**
 * a02yyuw.c
 *
 * Driver for the DFRobot A02YYUW waterproof ultrasonic distance sensor.
 *
 * Each call to a02yyuw_scan_driver() drains all pending UART data, parses
 * the most recent valid 4-byte frame from each sensor, and returns one
 * obstacle detection per sensor at that sensor's configured mounting angle.
 * This function is designed to be registered directly with obstacle_sense:
 *
 *   obstacle_sense_register_driver(a02yyuw_scan_driver);
 *
 * Multi-sensor example (4 sensors for 360° coverage):
 *
 *   a02yyuw_init(0, UART_NUM_1, GPIO_NUM_17, -1,   0);   // Forward
 *   a02yyuw_init(1, UART_NUM_2, GPIO_NUM_18, -1,  90);   // Starboard
 *   // sensors 2 and 3 would require additional UART or GPIO-based approaches
 *   obstacle_sense_register_driver(a02yyuw_scan_driver);
 */

#include "a02yyuw.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static const char *TAG = "A02YYUW";

// ============================================================================
// INTERNAL STATE
// ============================================================================

static a02yyuw_instance_t instances[A02YYUW_MAX_SENSORS];
static bool driver_initialized = false;

// ============================================================================
// FRAME PARSING
// ============================================================================

/**
 * Search a byte buffer for the last valid A02YYUW frame.
 *
 * Scans from the beginning looking for the 0xFF header. Validates the
 * checksum and range bounds. Returns the last (most recent) valid frame
 * found, since the sensor streams continuously and older frames may be
 * stacked in the buffer.
 *
 * @param buf       Buffer containing raw UART bytes
 * @param len       Number of bytes in buffer
 * @param range_out Output: parsed range in mm
 * @return          true if at least one valid frame was found
 */
static bool parse_latest_frame(const uint8_t *buf, int len, int16_t *range_out)
{
    bool found = false;

    for (int i = 0; i + (A02YYUW_FRAME_LEN - 1) < len; i++) {
        if (buf[i] != A02YYUW_FRAME_HEADER) {
            continue;
        }

        uint8_t h   = buf[i + 1];
        uint8_t l   = buf[i + 2];
        uint8_t sum = buf[i + 3];

        uint8_t expected = (uint8_t)((A02YYUW_FRAME_HEADER + h + l) & 0xFF);
        if (sum != expected) {
            continue;
        }

        int16_t range = (int16_t)(((uint16_t)h << 8) | l);
        if (range < A02YYUW_RANGE_MIN_MM || range > A02YYUW_RANGE_MAX_MM) {
            continue;
        }

        *range_out = range;
        found = true;
        /* Do not break — keep scanning to find the most recent valid frame. */
    }

    return found;
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t a02yyuw_init(int instance_index, uart_port_t uart_port,
                        int rx_pin, int tx_pin, int16_t mounting_angle_deg)
{
    if (instance_index < 0 || instance_index >= A02YYUW_MAX_SENSORS) {
        ESP_LOGE(TAG, "Invalid instance index %d (max %d)",
                 instance_index, A02YYUW_MAX_SENSORS - 1);
        return ESP_ERR_INVALID_ARG;
    }

    if (instances[instance_index].initialized) {
        ESP_LOGW(TAG, "Instance %d already initialized, re-initializing", instance_index);
        uart_driver_delete(instances[instance_index].uart_port);
    }

    /* Configure UART peripheral. */
    const uart_config_t uart_cfg = {
        .baud_rate  = A02YYUW_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(uart_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed (port %d): %s",
                 uart_port, esp_err_to_name(err));
        return err;
    }

    /* Assign GPIO pins. Pass UART_PIN_NO_CHANGE for unused TX. */
    int actual_tx = (tx_pin < 0) ? UART_PIN_NO_CHANGE : tx_pin;
    err = uart_set_pin(uart_port, actual_tx, rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed (port %d): %s",
                 uart_port, esp_err_to_name(err));
        return err;
    }

    /* Install UART driver with RX-only ring buffer. No event queue needed. */
    err = uart_driver_install(uart_port, A02YYUW_UART_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed (port %d): %s",
                 uart_port, esp_err_to_name(err));
        return err;
    }

    instances[instance_index].uart_port          = uart_port;
    instances[instance_index].rx_pin             = rx_pin;
    instances[instance_index].tx_pin             = tx_pin;
    instances[instance_index].mounting_angle_deg = mounting_angle_deg;
    instances[instance_index].last_range_mm      = 0;
    instances[instance_index].last_update_ms     = 0;
    instances[instance_index].initialized        = true;

    driver_initialized = true;

    ESP_LOGI(TAG, "Sensor %d ready: UART%d RX=GPIO%d TX=%s angle=%+d°",
             instance_index, (int)uart_port, rx_pin,
             (tx_pin < 0) ? "N/A" : "configured",
             mounting_angle_deg);

    return ESP_OK;
}

void a02yyuw_update_all(void)
{
    uint8_t buf[A02YYUW_UART_BUF_SIZE];
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    for (int i = 0; i < A02YYUW_MAX_SENSORS; i++) {
        if (!instances[i].initialized) {
            continue;
        }

        /* Non-blocking read — take whatever is waiting in the buffer. */
        int bytes = uart_read_bytes(instances[i].uart_port,
                                    buf, sizeof(buf) - 1,
                                    pdMS_TO_TICKS(0));
        if (bytes < A02YYUW_FRAME_LEN) {
            continue;
        }

        int16_t range = 0;
        if (parse_latest_frame(buf, bytes, &range)) {
            instances[i].last_range_mm  = range;
            instances[i].last_update_ms = now_ms;
            ESP_LOGV(TAG, "Sensor %d: %d mm @ %+d°",
                     i, range, instances[i].mounting_angle_deg);
        }
    }
}

int16_t a02yyuw_read_range(int instance_index)
{
    if (instance_index < 0 || instance_index >= A02YYUW_MAX_SENSORS) {
        return 0;
    }
    if (!instances[instance_index].initialized) {
        return 0;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t age    = now_ms - instances[instance_index].last_update_ms;

    if (age > A02YYUW_STALE_MS) {
        return 0;
    }

    return instances[instance_index].last_range_mm;
}

bool a02yyuw_scan_driver(obstacle_scan_t *scan)
{
    /* Pull latest data from all UART buffers before building scan. */
    a02yyuw_update_all();

    scan->num_detections = 0;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    for (int i = 0; i < A02YYUW_MAX_SENSORS; i++) {
        if (!instances[i].initialized) {
            continue;
        }
        if (scan->num_detections >= OBSTACLE_MAX_DETECTIONS) {
            break;
        }

        uint32_t age = now_ms - instances[i].last_update_ms;
        if (instances[i].last_range_mm <= 0 || age > A02YYUW_STALE_MS) {
            continue;  /* No fresh reading from this sensor. */
        }

        int d = scan->num_detections++;
        scan->detections[d].range_mm      = instances[i].last_range_mm;
        scan->detections[d].bearing_deg   = instances[i].mounting_angle_deg;
        scan->detections[d].velocity_mms  = 0;  /* A02YYUW does not measure velocity. */
        scan->detections[d].confidence    = A02YYUW_CONFIDENCE;
        scan->detections[d].object_type   = OBSTACLE_TYPE_UNKNOWN;
    }

    return true;
}

int a02yyuw_get_count(void)
{
    int count = 0;
    for (int i = 0; i < A02YYUW_MAX_SENSORS; i++) {
        if (instances[i].initialized) {
            count++;
        }
    }
    return count;
}

const a02yyuw_instance_t *a02yyuw_get_instance(int instance_index)
{
    if (instance_index < 0 || instance_index >= A02YYUW_MAX_SENSORS) {
        return NULL;
    }
    return &instances[instance_index];
}

void a02yyuw_print_status(void)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    printf("\n=== A02YYUW Ultrasonic Sensor Status ===\n");

    int count = a02yyuw_get_count();
    if (count == 0) {
        printf("  No sensors initialized.\n");
        printf("  Call a02yyuw_init() then obstacle_sense_register_driver(a02yyuw_scan_driver)\n");
        printf("\n");
        return;
    }

    printf("  %-8s %-8s %-10s %-12s %-10s %-8s\n",
           "Slot", "UART", "Angle", "Range (mm)", "Age (ms)", "Status");
    printf("  %-8s %-8s %-10s %-12s %-10s %-8s\n",
           "--------", "--------", "----------", "------------", "----------", "--------");

    for (int i = 0; i < A02YYUW_MAX_SENSORS; i++) {
        if (!instances[i].initialized) {
            continue;
        }
        uint32_t age = now_ms - instances[i].last_update_ms;
        bool stale   = (age > A02YYUW_STALE_MS) || (instances[i].last_range_mm <= 0);
        int16_t disp_range = stale ? 0 : instances[i].last_range_mm;

        printf("  %-8d UART%-4d %+6d°     %-12d %-10lu %s\n",
               i,
               (int)instances[i].uart_port,
               instances[i].mounting_angle_deg,
               disp_range,
               (unsigned long)age,
               stale ? "STALE" : "OK");
    }
    printf("\n");
}
