/**
 * obstacle_sense.c
 * 
 * Obstacle sensor interface with driver abstraction.
 * Step 7: Scaffolding with mock data for testing.
 * 
 * Architecture:
 * - Sensor driver is called at fixed interval to get detections
 * - Local scans are broadcast to swarm via MSG_TYPE_OBSTACLE
 * - Remote scans are stored for fusion/avoidance algorithms
 * - Mock driver allows testing without hardware
 */

#include "obstacle_sense.h"
#include "node_config.h"
#include "swarm_protocol.h"
#include "swarm_transport.h"
#include "esp_log.h"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "OBSTACLE_SENSE";

// ============================================================================
// OBSTACLE MESSAGE FORMAT
// ============================================================================

// Compact detection for transmission (8 bytes each)
typedef struct __attribute__((packed)) {
    int16_t range_mm;
    int16_t bearing_deg;
    int16_t velocity_mms;
    uint8_t confidence;
    uint8_t object_type;
} obstacle_detection_packed_t;

typedef struct __attribute__((packed)) {
    uint16_t source_node_id;
    uint8_t num_detections;
    uint8_t sensor_id;
    uint32_t timestamp_ms;
    uint32_t scan_seq;
    int16_t range_max_mm;
    int16_t fov_deg;
    // Followed by obstacle_detection_packed_t array
} obstacle_payload_t;

// ============================================================================
// INTERNAL STATE
// ============================================================================

static bool initialized = false;
static bool running = false;
static sensor_driver_fn current_driver = NULL;

// Local scan state
static obstacle_scan_t local_scan;
static uint32_t scan_count = 0;
static uint32_t broadcast_count = 0;
static uint32_t scan_seq = 0;

// Remote scans storage
typedef struct {
    obstacle_scan_t scan;
    bool valid;
} remote_scan_entry_t;

static remote_scan_entry_t remote_scans[OBSTACLE_MAX_NODES];

// Configuration
static int16_t config_range_max_mm = OBSTACLE_DEFAULT_RANGE_MAX_MM;
static int16_t config_fov_deg = 120;  // Default 120° FOV

// Mock mode state
static bool mock_enabled = true;  // Start in mock mode
static obstacle_detection_t mock_obstacles[OBSTACLE_MAX_DETECTIONS];
static int mock_obstacle_count = 0;

// Thread safety
static SemaphoreHandle_t scan_mutex = NULL;

// Callbacks
static obstacle_scan_cb_t scan_callback = NULL;
static obstacle_remote_cb_t remote_callback = NULL;

// Task handles
static TaskHandle_t scan_task_handle = NULL;
static TaskHandle_t broadcast_task_handle = NULL;

// ============================================================================
// MOCK DRIVER
// ============================================================================

static bool mock_driver(obstacle_scan_t *scan)
{
    scan->num_detections = 0;
    
    if (!mock_enabled || mock_obstacle_count == 0) {
        return true;  // Empty scan is valid
    }
    
    // Copy mock obstacles to scan
    for (int i = 0; i < mock_obstacle_count && i < OBSTACLE_MAX_DETECTIONS; i++) {
        scan->detections[i] = mock_obstacles[i];
        scan->num_detections++;
    }
    
    return true;
}

// ============================================================================
// PROTOCOL
// ============================================================================

static void broadcast_scan(const obstacle_scan_t *scan)
{
    uint16_t my_node_id = node_config_get_node_id();
    
    // Calculate payload size
    int detections_size = scan->num_detections * sizeof(obstacle_detection_packed_t);
    int payload_size = sizeof(obstacle_payload_t) + detections_size;
    
    // Build payload
    uint8_t payload[payload_size];
    obstacle_payload_t *hdr = (obstacle_payload_t *)payload;
    
    hdr->source_node_id = my_node_id;
    hdr->num_detections = scan->num_detections;
    hdr->sensor_id = scan->sensor_id;
    hdr->timestamp_ms = scan->timestamp_ms;
    hdr->scan_seq = scan->scan_seq;
    hdr->range_max_mm = scan->sensor_range_max_mm;
    hdr->fov_deg = scan->sensor_fov_deg;
    
    // Pack detections
    obstacle_detection_packed_t *packed = (obstacle_detection_packed_t *)(payload + sizeof(obstacle_payload_t));
    for (int i = 0; i < scan->num_detections; i++) {
        packed[i].range_mm = scan->detections[i].range_mm;
        packed[i].bearing_deg = scan->detections[i].bearing_deg;
        packed[i].velocity_mms = scan->detections[i].velocity_mms;
        packed[i].confidence = scan->detections[i].confidence;
        packed[i].object_type = scan->detections[i].object_type;
    }
    
    // Build swarm header
    swarm_header_t header;
    header.swarm_id = 0;
    header.src_id = my_node_id;
    header.dst_id = SWARM_BROADCAST_ADDR;
    header.group_id = 0;
    header.msg_type = MSG_TYPE_OBSTACLE;
    header.seq = (uint16_t)(scan->scan_seq & 0xFFFF);
    header.flags = 0;
    header.payload_len = payload_size;
    
    // Combine and send
    uint8_t packet[sizeof(swarm_header_t) + payload_size];
    memcpy(packet, &header, sizeof(swarm_header_t));
    memcpy(packet + sizeof(swarm_header_t), payload, payload_size);
    
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    swarm_transport_send(broadcast_mac, packet, sizeof(packet));
    
    broadcast_count++;
    ESP_LOGD(TAG, "Broadcast scan: %d detections, seq=%lu", 
             scan->num_detections, (unsigned long)scan->scan_seq);
}

static void process_obstacle_message(const uint8_t *data, int len)
{
    if (len < sizeof(obstacle_payload_t)) {
        ESP_LOGW(TAG, "Obstacle message too short: %d", len);
        return;
    }
    
    obstacle_payload_t hdr;
    memcpy(&hdr, data, sizeof(obstacle_payload_t));
    
    // Ignore our own broadcasts
    uint16_t my_node_id = node_config_get_node_id();
    if (hdr.source_node_id == my_node_id) {
        return;
    }
    
    // Validate detection count
    int expected_size = sizeof(obstacle_payload_t) + 
                        (hdr.num_detections * sizeof(obstacle_detection_packed_t));
    if (len < expected_size) {
        ESP_LOGW(TAG, "Obstacle message incomplete: expected %d, got %d", expected_size, len);
        return;
    }
    
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    
    // Find or allocate slot for this node
    int slot = -1;
    for (int i = 0; i < OBSTACLE_MAX_NODES; i++) {
        if (remote_scans[i].valid && remote_scans[i].scan.source_node_id == hdr.source_node_id) {
            slot = i;
            break;
        }
    }
    
    if (slot < 0) {
        // Find empty slot
        for (int i = 0; i < OBSTACLE_MAX_NODES; i++) {
            if (!remote_scans[i].valid) {
                slot = i;
                break;
            }
        }
    }
    
    if (slot < 0) {
        xSemaphoreGive(scan_mutex);
        ESP_LOGW(TAG, "Remote scan storage full");
        return;
    }
    
    // Check sequence number (ignore old scans)
    if (remote_scans[slot].valid && 
        hdr.scan_seq <= remote_scans[slot].scan.scan_seq) {
        xSemaphoreGive(scan_mutex);
        return;
    }
    
    // Store scan
    obstacle_scan_t *scan = &remote_scans[slot].scan;
    scan->source_node_id = hdr.source_node_id;
    scan->num_detections = hdr.num_detections;
    scan->sensor_id = hdr.sensor_id;
    scan->timestamp_ms = hdr.timestamp_ms;
    scan->scan_seq = hdr.scan_seq;
    scan->sensor_range_max_mm = hdr.range_max_mm;
    scan->sensor_fov_deg = hdr.fov_deg;
    
    // Unpack detections
    const obstacle_detection_packed_t *packed = 
        (const obstacle_detection_packed_t *)(data + sizeof(obstacle_payload_t));
    for (int i = 0; i < hdr.num_detections && i < OBSTACLE_MAX_DETECTIONS; i++) {
        scan->detections[i].range_mm = packed[i].range_mm;
        scan->detections[i].bearing_deg = packed[i].bearing_deg;
        scan->detections[i].velocity_mms = packed[i].velocity_mms;
        scan->detections[i].confidence = packed[i].confidence;
        scan->detections[i].object_type = packed[i].object_type;
    }
    
    remote_scans[slot].valid = true;
    
    // Copy for callback
    obstacle_scan_t scan_copy = *scan;
    
    xSemaphoreGive(scan_mutex);
    
    ESP_LOGD(TAG, "Received scan from node %u: %d detections", 
             hdr.source_node_id, hdr.num_detections);
    
    // Notify callback
    if (remote_callback) {
        remote_callback(hdr.source_node_id, &scan_copy);
    }
}

// ============================================================================
// TRANSPORT RECEIVE HANDLER
// ============================================================================

static void obstacle_recv_handler(const uint8_t *src_mac, const uint8_t *data,
                                  int len, int8_t rssi)
{
    if (len < sizeof(swarm_header_t)) {
        return;
    }
    
    swarm_header_t header;
    memcpy(&header, data, sizeof(swarm_header_t));
    
    if (header.msg_type != MSG_TYPE_OBSTACLE) {
        return;
    }
    
    const uint8_t *payload = data + sizeof(swarm_header_t);
    int payload_len = len - sizeof(swarm_header_t);
    
    process_obstacle_message(payload, payload_len);
}

// ============================================================================
// SCAN TASK
// ============================================================================

static void perform_scan(void)
{
    if (!current_driver) {
        return;
    }
    
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    
    // Clear previous detections
    memset(&local_scan, 0, sizeof(local_scan));
    local_scan.source_node_id = node_config_get_node_id();
    local_scan.sensor_id = 0;
    local_scan.timestamp_ms = esp_log_timestamp();
    local_scan.scan_seq = ++scan_seq;
    local_scan.sensor_range_max_mm = config_range_max_mm;
    local_scan.sensor_fov_deg = config_fov_deg;
    
    // Call driver
    bool success = current_driver(&local_scan);
    
    if (success) {
        scan_count++;
    }
    
    // Copy for callback
    obstacle_scan_t scan_copy = local_scan;
    
    xSemaphoreGive(scan_mutex);
    
    if (success && scan_callback) {
        scan_callback(&scan_copy);
    }
}

static void scan_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Scan task started (interval=%d ms)", OBSTACLE_SCAN_INTERVAL_MS);
    
    while (running) {
        perform_scan();
        vTaskDelay(pdMS_TO_TICKS(OBSTACLE_SCAN_INTERVAL_MS));
    }
    
    ESP_LOGI(TAG, "Scan task stopped");
    vTaskDelete(NULL);
}

static void broadcast_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Broadcast task started (interval=%d ms)", OBSTACLE_BROADCAST_INTERVAL_MS);
    
    while (running) {
        xSemaphoreTake(scan_mutex, portMAX_DELAY);
        obstacle_scan_t scan_copy = local_scan;
        xSemaphoreGive(scan_mutex);
        
        if (scan_copy.scan_seq > 0) {
            broadcast_scan(&scan_copy);
        }
        
        vTaskDelay(pdMS_TO_TICKS(OBSTACLE_BROADCAST_INTERVAL_MS));
    }
    
    ESP_LOGI(TAG, "Broadcast task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t obstacle_sense_init(void)
{
    if (initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing obstacle sensing...");
    
    // Create mutex
    if (scan_mutex == NULL) {
        scan_mutex = xSemaphoreCreateMutex();
        if (scan_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Clear state
    memset(&local_scan, 0, sizeof(local_scan));
    memset(remote_scans, 0, sizeof(remote_scans));
    memset(mock_obstacles, 0, sizeof(mock_obstacles));
    mock_obstacle_count = 0;
    scan_count = 0;
    broadcast_count = 0;
    scan_seq = 0;
    
    // Register mock driver by default
    current_driver = mock_driver;
    mock_enabled = true;
    
    initialized = true;
    ESP_LOGI(TAG, "Obstacle sensing initialized (mock mode enabled)");
    return ESP_OK;
}

esp_err_t obstacle_sense_start(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (running) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }
    
    uint16_t my_node_id = node_config_get_node_id();
    if (my_node_id == NODE_ID_INVALID) {
        ESP_LOGW(TAG, "Cannot start - node not provisioned");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting obstacle sensing...");
    
    // Register receive handler
    swarm_transport_register_recv_cb(obstacle_recv_handler);
    
    running = true;
    
    // Start tasks
    xTaskCreate(scan_task, "obs_scan", 4096, NULL, 4, &scan_task_handle);
    xTaskCreate(broadcast_task, "obs_bcast", 3072, NULL, 3, &broadcast_task_handle);
    
    ESP_LOGI(TAG, "Obstacle sensing started (range=%d mm, fov=%d°)", 
             config_range_max_mm, config_fov_deg);
    return ESP_OK;
}

void obstacle_sense_stop(void)
{
    if (!running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping obstacle sensing...");
    running = false;
    
    scan_task_handle = NULL;
    broadcast_task_handle = NULL;
    
    ESP_LOGI(TAG, "Obstacle sensing stopped");
}

esp_err_t obstacle_sense_register_driver(sensor_driver_fn driver)
{
    if (driver == NULL) {
        ESP_LOGE(TAG, "Invalid driver");
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    current_driver = driver;
    mock_enabled = false;  // Disable mock when real driver registered
    xSemaphoreGive(scan_mutex);
    
    ESP_LOGI(TAG, "Custom sensor driver registered");
    return ESP_OK;
}

void obstacle_sense_configure(int16_t range_max_mm, int16_t fov_deg)
{
    config_range_max_mm = range_max_mm;
    config_fov_deg = fov_deg;
    ESP_LOGI(TAG, "Configured: range=%d mm, fov=%d°", range_max_mm, fov_deg);
}

esp_err_t obstacle_sense_get_local_scan(obstacle_scan_t *scan)
{
    if (scan == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    
    if (local_scan.scan_seq == 0) {
        xSemaphoreGive(scan_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    *scan = local_scan;
    xSemaphoreGive(scan_mutex);
    
    return ESP_OK;
}

esp_err_t obstacle_sense_get_node_scan(uint16_t node_id, obstacle_scan_t *scan)
{
    if (scan == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if requesting local scan
    if (node_id == node_config_get_node_id()) {
        return obstacle_sense_get_local_scan(scan);
    }
    
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    
    for (int i = 0; i < OBSTACLE_MAX_NODES; i++) {
        if (remote_scans[i].valid && remote_scans[i].scan.source_node_id == node_id) {
            *scan = remote_scans[i].scan;
            xSemaphoreGive(scan_mutex);
            return ESP_OK;
        }
    }
    
    xSemaphoreGive(scan_mutex);
    return ESP_ERR_NOT_FOUND;
}

int obstacle_sense_get_all_scans(obstacle_scan_t *scans, int max_scans)
{
    if (scans == NULL || max_scans <= 0) {
        return 0;
    }
    
    int count = 0;
    
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    
    // Add local scan first
    if (local_scan.scan_seq > 0 && count < max_scans) {
        scans[count++] = local_scan;
    }
    
    // Add remote scans
    uint32_t now = esp_log_timestamp();
    for (int i = 0; i < OBSTACLE_MAX_NODES && count < max_scans; i++) {
        if (remote_scans[i].valid) {
            // Check if stale
            uint32_t age = now - remote_scans[i].scan.timestamp_ms;
            if (age < OBSTACLE_SCAN_TIMEOUT_MS) {
                scans[count++] = remote_scans[i].scan;
            }
        }
    }
    
    xSemaphoreGive(scan_mutex);
    return count;
}

esp_err_t obstacle_sense_get_closest(obstacle_detection_t *detection, uint16_t *node_id)
{
    if (detection == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    obstacle_scan_t scans[OBSTACLE_MAX_NODES + 1];
    int num_scans = obstacle_sense_get_all_scans(scans, OBSTACLE_MAX_NODES + 1);
    
    int16_t closest_range = INT16_MAX;
    bool found = false;
    
    for (int s = 0; s < num_scans; s++) {
        for (int d = 0; d < scans[s].num_detections; d++) {
            if (scans[s].detections[d].range_mm > 0 && 
                scans[s].detections[d].range_mm < closest_range) {
                closest_range = scans[s].detections[d].range_mm;
                *detection = scans[s].detections[d];
                if (node_id) {
                    *node_id = scans[s].source_node_id;
                }
                found = true;
            }
        }
    }
    
    return found ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool obstacle_sense_path_clear(int16_t bearing_deg, int16_t corridor_deg, int16_t range_mm)
{
    int16_t half_corridor = corridor_deg / 2;
    int16_t min_bearing = bearing_deg - half_corridor;
    int16_t max_bearing = bearing_deg + half_corridor;
    
    obstacle_scan_t scans[OBSTACLE_MAX_NODES + 1];
    int num_scans = obstacle_sense_get_all_scans(scans, OBSTACLE_MAX_NODES + 1);
    
    for (int s = 0; s < num_scans; s++) {
        for (int d = 0; d < scans[s].num_detections; d++) {
            obstacle_detection_t *det = &scans[s].detections[d];
            
            // Check if in range
            if (det->range_mm <= 0 || det->range_mm > range_mm) {
                continue;
            }
            
            // Check if in corridor (handle wraparound at ±180)
            int16_t det_bearing = det->bearing_deg;
            
            // Normalize bearings
            while (det_bearing < -180) det_bearing += 360;
            while (det_bearing > 180) det_bearing -= 360;
            
            if (det_bearing >= min_bearing && det_bearing <= max_bearing) {
                return false;  // Obstacle in path
            }
        }
    }
    
    return true;  // Path clear
}

void obstacle_sense_get_status(obstacle_sense_status_t *status)
{
    if (status == NULL) {
        return;
    }
    
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    
    status->driver_registered = (current_driver != NULL);
    status->running = running;
    status->mock_enabled = mock_enabled;
    status->scan_count = scan_count;
    status->broadcast_count = broadcast_count;
    status->last_scan_ms = local_scan.timestamp_ms;
    status->range_max_mm = config_range_max_mm;
    status->fov_deg = config_fov_deg;
    
    xSemaphoreGive(scan_mutex);
}

void obstacle_sense_register_scan_cb(obstacle_scan_cb_t cb)
{
    scan_callback = cb;
}

void obstacle_sense_register_remote_cb(obstacle_remote_cb_t cb)
{
    remote_callback = cb;
}

// ============================================================================
// MOCK MODE
// ============================================================================

void obstacle_sense_mock_enable(bool enable)
{
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    mock_enabled = enable;
    if (enable) {
        current_driver = mock_driver;
    }
    xSemaphoreGive(scan_mutex);
    
    ESP_LOGI(TAG, "Mock mode %s", enable ? "enabled" : "disabled");
}

bool obstacle_sense_mock_is_enabled(void)
{
    return mock_enabled;
}

esp_err_t obstacle_sense_mock_add(int16_t range_mm, int16_t bearing_deg,
                                   int16_t velocity_mms, obstacle_type_t type)
{
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    
    if (mock_obstacle_count >= OBSTACLE_MAX_DETECTIONS) {
        xSemaphoreGive(scan_mutex);
        ESP_LOGW(TAG, "Mock obstacle list full");
        return ESP_ERR_NO_MEM;
    }
    
    mock_obstacles[mock_obstacle_count].range_mm = range_mm;
    mock_obstacles[mock_obstacle_count].bearing_deg = bearing_deg;
    mock_obstacles[mock_obstacle_count].velocity_mms = velocity_mms;
    mock_obstacles[mock_obstacle_count].confidence = 90;
    mock_obstacles[mock_obstacle_count].object_type = type;
    mock_obstacle_count++;
    
    xSemaphoreGive(scan_mutex);
    
    ESP_LOGI(TAG, "Mock obstacle added: range=%d mm, bearing=%d°", range_mm, bearing_deg);
    return ESP_OK;
}

esp_err_t obstacle_sense_mock_add_simple(int16_t range_mm, int16_t bearing_deg)
{
    return obstacle_sense_mock_add(range_mm, bearing_deg, 0, OBSTACLE_TYPE_UNKNOWN);
}

void obstacle_sense_mock_clear(void)
{
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    memset(mock_obstacles, 0, sizeof(mock_obstacles));
    mock_obstacle_count = 0;
    xSemaphoreGive(scan_mutex);
    
    ESP_LOGI(TAG, "Mock obstacles cleared");
}

int obstacle_sense_mock_count(void)
{
    return mock_obstacle_count;
}

// ============================================================================
// UTILITIES
// ============================================================================

const char* obstacle_type_to_string(obstacle_type_t type)
{
    switch (type) {
        case OBSTACLE_TYPE_UNKNOWN: return "UNKNOWN";
        case OBSTACLE_TYPE_STATIC:  return "STATIC";
        case OBSTACLE_TYPE_MOVING:  return "MOVING";
        case OBSTACLE_TYPE_BOAT:    return "BOAT";
        case OBSTACLE_TYPE_SHORE:   return "SHORE";
        case OBSTACLE_TYPE_DEBRIS:  return "DEBRIS";
        default: return "?";
    }
}

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

static int cmd_sensors(int argc, char **argv)
{
    obstacle_sense_print_status();
    return 0;
}

static int cmd_detections(int argc, char **argv)
{
    obstacle_sense_print_detections();
    return 0;
}

static int cmd_mock_add(int argc, char **argv)
{
    if (argc < 3) {
        printf("Usage: mock_add <range_mm> <bearing_deg> [velocity_mms] [type]\n");
        printf("  type: 0=unknown, 1=static, 2=moving, 3=boat, 4=shore, 5=debris\n");
        return 1;
    }
    
    int16_t range = atoi(argv[1]);
    int16_t bearing = atoi(argv[2]);
    int16_t velocity = (argc >= 4) ? atoi(argv[3]) : 0;
    obstacle_type_t type = (argc >= 5) ? (obstacle_type_t)atoi(argv[4]) : OBSTACLE_TYPE_UNKNOWN;
    
    esp_err_t err = obstacle_sense_mock_add(range, bearing, velocity, type);
    if (err == ESP_OK) {
        printf("Mock obstacle added: %d mm @ %d°\n", range, bearing);
    } else {
        printf("Failed to add mock obstacle: %s\n", esp_err_to_name(err));
    }
    
    return 0;
}

static int cmd_mock_clear(int argc, char **argv)
{
    obstacle_sense_mock_clear();
    printf("Mock obstacles cleared\n");
    return 0;
}

static int cmd_path_check(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: path_check <bearing_deg> [corridor_deg] [range_mm]\n");
        printf("  Default corridor: 30°, range: 3000mm\n");
        return 1;
    }
    
    int16_t bearing = atoi(argv[1]);
    int16_t corridor = (argc >= 3) ? atoi(argv[2]) : 30;
    int16_t range = (argc >= 4) ? atoi(argv[3]) : 3000;
    
    bool clear = obstacle_sense_path_clear(bearing, corridor, range);
    printf("Path %d° (±%d°) to %d mm: %s\n", 
           bearing, corridor/2, range, clear ? "CLEAR" : "BLOCKED");
    
    return 0;
}

void obstacle_sense_register_commands(void)
{
    const esp_console_cmd_t sensors_cmd = {
        .command = "sensors",
        .help = "Show sensor status",
        .hint = NULL,
        .func = &cmd_sensors,
    };
    esp_console_cmd_register(&sensors_cmd);
    
    const esp_console_cmd_t detections_cmd = {
        .command = "detections",
        .help = "Show all detected obstacles",
        .hint = NULL,
        .func = &cmd_detections,
    };
    esp_console_cmd_register(&detections_cmd);
    
    const esp_console_cmd_t mock_add_cmd = {
        .command = "mock_add",
        .help = "Add mock obstacle: mock_add <range_mm> <bearing_deg>",
        .hint = NULL,
        .func = &cmd_mock_add,
    };
    esp_console_cmd_register(&mock_add_cmd);
    
    const esp_console_cmd_t mock_clear_cmd = {
        .command = "mock_clear",
        .help = "Clear all mock obstacles",
        .hint = NULL,
        .func = &cmd_mock_clear,
    };
    esp_console_cmd_register(&mock_clear_cmd);
    
    const esp_console_cmd_t path_check_cmd = {
        .command = "path_check",
        .help = "Check if path is clear: path_check <bearing_deg>",
        .hint = NULL,
        .func = &cmd_path_check,
    };
    esp_console_cmd_register(&path_check_cmd);
}

void obstacle_sense_print_status(void)
{
    obstacle_sense_status_t status;
    obstacle_sense_get_status(&status);
    
    printf("\n=== Obstacle Sensor Status ===\n");
    printf("Driver registered: %s\n", status.driver_registered ? "YES" : "NO");
    printf("Running: %s\n", status.running ? "YES" : "NO");
    printf("Mock mode: %s\n", status.mock_enabled ? "ENABLED" : "DISABLED");
    printf("Mock obstacles: %d\n", mock_obstacle_count);
    printf("Range max: %d mm\n", status.range_max_mm);
    printf("Field of view: %d°\n", status.fov_deg);
    printf("Scan count: %lu\n", (unsigned long)status.scan_count);
    printf("Broadcast count: %lu\n", (unsigned long)status.broadcast_count);
    
    if (status.last_scan_ms > 0) {
        uint32_t age = esp_log_timestamp() - status.last_scan_ms;
        printf("Last scan: %lu ms ago\n", (unsigned long)age);
    }
    printf("\n");
}

void obstacle_sense_print_detections(void)
{
    obstacle_scan_t scans[OBSTACLE_MAX_NODES + 1];
    int num_scans = obstacle_sense_get_all_scans(scans, OBSTACLE_MAX_NODES + 1);
    
    printf("\n=== Obstacle Detections ===\n");
    
    if (num_scans == 0) {
        printf("No scans available\n\n");
        return;
    }
    
    int total_detections = 0;
    
    for (int s = 0; s < num_scans; s++) {
        uint32_t age = esp_log_timestamp() - scans[s].timestamp_ms;
        bool is_local = (scans[s].source_node_id == node_config_get_node_id());
        
        printf("\n--- Node %u %s (age: %lu ms) ---\n",
               scans[s].source_node_id,
               is_local ? "(local)" : "(remote)",
               (unsigned long)age);
        
        if (scans[s].num_detections == 0) {
            printf("  No obstacles detected\n");
            continue;
        }
        
        printf("  %-10s %-10s %-10s %-8s %-10s\n", 
               "Range", "Bearing", "Velocity", "Conf", "Type");
        printf("  --------------------------------------------------\n");
        
        for (int d = 0; d < scans[s].num_detections; d++) {
            obstacle_detection_t *det = &scans[s].detections[d];
            printf("  %-10d %-10d %-10d %-8d %-10s\n",
                   det->range_mm,
                   det->bearing_deg,
                   det->velocity_mms,
                   det->confidence,
                   obstacle_type_to_string(det->object_type));
            total_detections++;
        }
    }
    
    printf("\nTotal: %d detections from %d nodes\n\n", total_detections, num_scans);
    
    // Show closest obstacle
    obstacle_detection_t closest;
    uint16_t closest_node;
    if (obstacle_sense_get_closest(&closest, &closest_node) == ESP_OK) {
        printf("Closest obstacle: %d mm @ %d° (from node %u)\n\n",
               closest.range_mm, closest.bearing_deg, closest_node);
    }
}
