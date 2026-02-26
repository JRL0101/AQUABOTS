/**
 * formation.c
 * 
 * Formation control with extensible pattern system.
 * Coordinated spatial formations: line, wedge, circle, column.
 * 
 * Architecture:
 * - Pattern calculators are registered as function pointers
 * - Leader broadcasts formation config via MSG_TYPE_FORMATION
 * - Each node calculates its own position from config
 * - Adding new patterns requires only a new calculator function
 */

#include "formation.h"
#include "leader_election.h"
#include "membership.h"
#include "node_config.h"
#include "swarm_protocol.h"
#include "swarm_transport.h"
#include "esp_log.h"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>
#include <ctype.h>

static const char *TAG = "FORMATION";

// ============================================================================
// FORMATION MESSAGE FORMAT
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t formation_type;     // formation_type_t
    uint8_t node_count;         // Number of nodes
    uint16_t reserved;          // Padding
    int32_t spacing_mm;         // Spacing between nodes
    int32_t center_x_mm;        // Formation center X
    int32_t center_y_mm;        // Formation center Y
    int16_t center_heading_deg; // Formation heading
    uint16_t reserved2;         // Padding
    uint32_t config_seq;        // Sequence number
} formation_payload_t;

// ============================================================================
// INTERNAL STATE
// ============================================================================

static formation_config_t current_config;
static formation_position_t my_position;
static int my_formation_index = -1;
static bool position_valid = false;
static bool running = false;

// Pattern registry
static pattern_entry_t pattern_registry[FORMATION_MAX_PATTERNS];
static int registered_patterns = 0;

// Thread safety
static SemaphoreHandle_t formation_mutex = NULL;

// Callbacks
static formation_changed_cb_t changed_callback = NULL;
static position_assigned_cb_t position_callback = NULL;

// ============================================================================
// BUILT-IN PATTERN CALCULATORS
// ============================================================================

/**
 * LINE formation: Single file, leader at front
 * 
 *     [0] Leader
 *      |
 *     [1]
 *      |
 *     [2]
 *      |
 *     ...
 */
static bool pattern_line_calculate(int node_index, int total_nodes,
                                   int32_t spacing_mm, formation_position_t *pos)
{
    // Leader at (0, 0), followers spaced behind (-Y direction)
    pos->x_mm = 0;
    pos->y_mm = -node_index * spacing_mm;
    pos->heading_deg = 0;  // All facing same direction
    return true;
}

/**
 * WEDGE formation: V-shape with leader at point
 * 
 *         [0] Leader
 *        /   \
 *      [1]   [2]
 *      /       \
 *    [3]       [4]
 */
static bool pattern_wedge_calculate(int node_index, int total_nodes,
                                    int32_t spacing_mm, formation_position_t *pos)
{
    if (node_index == 0) {
        // Leader at point
        pos->x_mm = 0;
        pos->y_mm = 0;
        pos->heading_deg = 0;
        return true;
    }
    
    // Calculate row and position within row
    // Row 1: indices 1,2 | Row 2: indices 3,4 | Row 3: indices 5,6 | etc.
    int row = (node_index + 1) / 2;  // 1->1, 2->1, 3->2, 4->2, etc.
    bool left_side = (node_index % 2 == 1);  // Odd indices on left
    
    // Y offset (behind leader)
    pos->y_mm = -row * spacing_mm;
    
    // X offset (spread out from center)
    int32_t x_offset = row * spacing_mm / 2;  // Angle ~30 degrees
    pos->x_mm = left_side ? -x_offset : x_offset;
    
    pos->heading_deg = 0;  // All facing same direction
    return true;
}

/**
 * CIRCLE formation: Nodes distributed around center
 * 
 *       [1]
 *     /     \
 *   [4]     [2]
 *     \     /
 *       [3]
 *   
 *   [0] at center (leader)
 */
static bool pattern_circle_calculate(int node_index, int total_nodes,
                                     int32_t spacing_mm, formation_position_t *pos)
{
    if (total_nodes <= 1) {
        pos->x_mm = 0;
        pos->y_mm = 0;
        pos->heading_deg = 0;
        return true;
    }
    
    if (node_index == 0) {
        // Leader at center
        pos->x_mm = 0;
        pos->y_mm = 0;
        pos->heading_deg = 0;
        return true;
    }
    
    // Calculate radius based on spacing and number of nodes on circle
    int nodes_on_circle = total_nodes - 1;
    
    // Circumference = nodes * spacing, radius = circumference / (2*pi)
    // Using integer math: radius = (nodes * spacing) / 6 (approximating 2*pi as 6)
    int32_t radius_mm = (nodes_on_circle * spacing_mm) / 6;
    if (radius_mm < spacing_mm) {
        radius_mm = spacing_mm;  // Minimum radius
    }
    
    // Angle for this node (evenly distributed)
    // node_index 1 -> 0 degrees (top), going clockwise
    float angle_rad = ((node_index - 1) * 2.0f * M_PI) / nodes_on_circle;
    
    pos->x_mm = (int32_t)(radius_mm * sinf(angle_rad));
    pos->y_mm = (int32_t)(radius_mm * cosf(angle_rad));
    
    // Face outward from center
    pos->heading_deg = (int16_t)((angle_rad * 180.0f) / M_PI);
    
    return true;
}

/**
 * COLUMN formation: Two-wide columns
 * 
 *   [0] [1]   <- Row 0
 *   [2] [3]   <- Row 1
 *   [4] [5]   <- Row 2
 *   ...
 */
static bool pattern_column_calculate(int node_index, int total_nodes,
                                     int32_t spacing_mm, formation_position_t *pos)
{
    int row = node_index / 2;
    int col = node_index % 2;
    
    // X offset: left column (-spacing/2), right column (+spacing/2)
    pos->x_mm = (col == 0) ? -(spacing_mm / 2) : (spacing_mm / 2);
    
    // Y offset: rows behind
    pos->y_mm = -row * spacing_mm;
    
    pos->heading_deg = 0;  // All facing same direction
    return true;
}

/**
 * NONE formation: All nodes at center (effectively disabled)
 */
static bool pattern_none_calculate(int node_index, int total_nodes,
                                   int32_t spacing_mm, formation_position_t *pos)
{
    pos->x_mm = 0;
    pos->y_mm = 0;
    pos->heading_deg = 0;
    return true;
}

// ============================================================================
// PATTERN REGISTRY
// ============================================================================

static void register_builtin_patterns(void)
{
    formation_register_pattern(FORMATION_NONE, "none", pattern_none_calculate);
    formation_register_pattern(FORMATION_LINE, "line", pattern_line_calculate);
    formation_register_pattern(FORMATION_WEDGE, "wedge", pattern_wedge_calculate);
    formation_register_pattern(FORMATION_CIRCLE, "circle", pattern_circle_calculate);
    formation_register_pattern(FORMATION_COLUMN, "column", pattern_column_calculate);
}

static pattern_entry_t* find_pattern(formation_type_t type)
{
    for (int i = 0; i < registered_patterns; i++) {
        if (pattern_registry[i].registered && pattern_registry[i].type == type) {
            return &pattern_registry[i];
        }
    }
    return NULL;
}

esp_err_t formation_register_pattern(formation_type_t type, const char *name,
                                     pattern_calculator_fn calculator)
{
    if (registered_patterns >= FORMATION_MAX_PATTERNS) {
        ESP_LOGE(TAG, "Pattern registry full");
        return ESP_ERR_NO_MEM;
    }
    
    // Check for duplicate
    if (find_pattern(type) != NULL) {
        ESP_LOGW(TAG, "Pattern type %d already registered, updating", type);
        pattern_entry_t *entry = find_pattern(type);
        entry->name = name;
        entry->calculator = calculator;
        return ESP_OK;
    }
    
    pattern_registry[registered_patterns].type = type;
    pattern_registry[registered_patterns].name = name;
    pattern_registry[registered_patterns].calculator = calculator;
    pattern_registry[registered_patterns].registered = true;
    registered_patterns++;
    
    ESP_LOGI(TAG, "Registered pattern: %s (type=%d)", name, type);
    return ESP_OK;
}

const char* formation_type_to_string(formation_type_t type)
{
    pattern_entry_t *entry = find_pattern(type);
    return entry ? entry->name : "UNKNOWN";
}

formation_type_t formation_type_from_string(const char *name)
{
    for (int i = 0; i < registered_patterns; i++) {
        if (pattern_registry[i].registered) {
            // Case-insensitive comparison
            const char *p1 = name;
            const char *p2 = pattern_registry[i].name;
            bool match = true;
            while (*p1 && *p2) {
                if (tolower((unsigned char)*p1) != tolower((unsigned char)*p2)) {
                    match = false;
                    break;
                }
                p1++;
                p2++;
            }
            if (match && *p1 == '\0' && *p2 == '\0') {
                return pattern_registry[i].type;
            }
        }
    }
    return FORMATION_NONE;
}

// ============================================================================
// POSITION CALCULATION
// ============================================================================

static int get_node_formation_index(uint16_t node_id)
{
    // Get all members sorted by node_id
    const member_info_t *members[MEMBERSHIP_MAX_MEMBERS];
    int count = membership_get_all_members(members, MEMBERSHIP_MAX_MEMBERS);
    
    uint16_t my_node_id = node_config_get_node_id();
    uint16_t leader_id = leader_election_get_leader();
    
    // Build sorted list of node IDs (leader first, then by node_id)
    uint16_t node_ids[MEMBERSHIP_MAX_MEMBERS + 1];
    int num_nodes = 0;
    
    // Leader is always index 0
    node_ids[num_nodes++] = leader_id;
    
    // Add other members (including self if not leader)
    for (int i = 0; i < count; i++) {
        if (members[i]->node_id != leader_id) {
            node_ids[num_nodes++] = members[i]->node_id;
        }
    }
    
    // Add self if not in member list and not leader
    bool self_found = (my_node_id == leader_id);
    for (int i = 0; i < count && !self_found; i++) {
        if (members[i]->node_id == my_node_id) {
            self_found = true;
        }
    }
    if (!self_found) {
        node_ids[num_nodes++] = my_node_id;
    }
    
    // Sort non-leader nodes by node_id (simple bubble sort, small array)
    for (int i = 1; i < num_nodes - 1; i++) {
        for (int j = 1; j < num_nodes - i; j++) {
            if (node_ids[j] > node_ids[j + 1]) {
                uint16_t temp = node_ids[j];
                node_ids[j] = node_ids[j + 1];
                node_ids[j + 1] = temp;
            }
        }
    }
    
    // Find target node's index
    for (int i = 0; i < num_nodes; i++) {
        if (node_ids[i] == node_id) {
            return i;
        }
    }
    
    return -1;  // Not found
}

static void recalculate_my_position(void)
{
    uint16_t my_node_id = node_config_get_node_id();
    
    // Get my index in formation
    my_formation_index = get_node_formation_index(my_node_id);
    
    if (my_formation_index < 0) {
        ESP_LOGW(TAG, "Could not determine formation index");
        position_valid = false;
        return;
    }
    
    // Find pattern calculator
    pattern_entry_t *pattern = find_pattern(current_config.type);
    if (!pattern || !pattern->calculator) {
        ESP_LOGW(TAG, "No calculator for formation type %d", current_config.type);
        position_valid = false;
        return;
    }
    
    // Calculate position
    formation_position_t new_pos;
    if (pattern->calculator(my_formation_index, current_config.node_count,
                           current_config.spacing_mm, &new_pos)) {
        my_position = new_pos;
        position_valid = true;
        
        ESP_LOGI(TAG, "Position calculated: index=%d, offset=(%ld, %ld) mm, hdg=%d°",
                 my_formation_index, 
                 (long)my_position.x_mm, (long)my_position.y_mm,
                 my_position.heading_deg);
        
        // Notify callback
        if (position_callback) {
            position_callback(&my_position, my_formation_index);
        }
    } else {
        ESP_LOGE(TAG, "Position calculation failed");
        position_valid = false;
    }
}

// ============================================================================
// FORMATION PROTOCOL
// ============================================================================

static void send_formation_config(void)
{
    if (!leader_election_is_leader()) {
        return;
    }
    
    uint16_t my_node_id = node_config_get_node_id();
    
    // Build payload
    formation_payload_t payload;
    payload.formation_type = (uint8_t)current_config.type;
    payload.node_count = current_config.node_count;
    payload.reserved = 0;
    payload.spacing_mm = current_config.spacing_mm;
    payload.center_x_mm = current_config.center_x_mm;
    payload.center_y_mm = current_config.center_y_mm;
    payload.center_heading_deg = current_config.center_heading_deg;
    payload.reserved2 = 0;
    payload.config_seq = current_config.config_seq;
    
    // Build swarm header
    swarm_header_t header;
    header.swarm_id = 0;
    header.src_id = my_node_id;
    header.dst_id = SWARM_BROADCAST_ADDR;
    header.group_id = 0;
    header.msg_type = MSG_TYPE_FORMATION;
    header.seq = (uint16_t)(current_config.config_seq & 0xFFFF);
    header.flags = 0;
    header.payload_len = sizeof(formation_payload_t);
    
    // Combine header + payload
    uint8_t packet[sizeof(swarm_header_t) + sizeof(formation_payload_t)];
    memcpy(packet, &header, sizeof(swarm_header_t));
    memcpy(packet + sizeof(swarm_header_t), &payload, sizeof(formation_payload_t));
    
    // Broadcast
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    swarm_transport_send(broadcast_mac, packet, sizeof(packet));
    
    ESP_LOGD(TAG, "Formation config broadcast: type=%s, nodes=%d, seq=%lu",
             formation_type_to_string(current_config.type),
             current_config.node_count,
             (unsigned long)current_config.config_seq);
}

static void process_formation_message(const uint8_t *src_mac, const uint8_t *data, int len)
{
    if (len < sizeof(formation_payload_t)) {
        ESP_LOGW(TAG, "Formation message too short: %d", len);
        return;
    }
    
    formation_payload_t payload;
    memcpy(&payload, data, sizeof(formation_payload_t));
    
    // Check sequence number (ignore old configs)
    if (payload.config_seq <= current_config.config_seq && current_config.config_seq > 0) {
        ESP_LOGD(TAG, "Ignoring old formation config (seq %lu <= %lu)",
                 (unsigned long)payload.config_seq, 
                 (unsigned long)current_config.config_seq);
        return;
    }
    
    xSemaphoreTake(formation_mutex, portMAX_DELAY);
    
    // Update config
    bool type_changed = (current_config.type != (formation_type_t)payload.formation_type);
    bool config_changed = type_changed ||
                         (current_config.node_count != payload.node_count) ||
                         (current_config.spacing_mm != payload.spacing_mm) ||
                         (current_config.center_x_mm != payload.center_x_mm) ||
                         (current_config.center_y_mm != payload.center_y_mm) ||
                         (current_config.center_heading_deg != payload.center_heading_deg);
    
    current_config.type = (formation_type_t)payload.formation_type;
    current_config.node_count = payload.node_count;
    current_config.spacing_mm = payload.spacing_mm;
    current_config.center_x_mm = payload.center_x_mm;
    current_config.center_y_mm = payload.center_y_mm;
    current_config.center_heading_deg = payload.center_heading_deg;
    current_config.config_seq = payload.config_seq;
    
    if (config_changed) {
        ESP_LOGI(TAG, "Formation config updated: type=%s, nodes=%d, spacing=%ld mm",
                 formation_type_to_string(current_config.type),
                 current_config.node_count,
                 (long)current_config.spacing_mm);
        
        // Recalculate my position
        recalculate_my_position();
        
        // Notify callback (outside mutex)
        formation_config_t config_copy = current_config;
        xSemaphoreGive(formation_mutex);
        
        if (changed_callback) {
            changed_callback(&config_copy);
        }
    } else {
        xSemaphoreGive(formation_mutex);
    }
}

// ============================================================================
// TRANSPORT RECEIVE HANDLER
// ============================================================================

static void formation_recv_handler(const uint8_t *src_mac, const uint8_t *data,
                                   int len, int8_t rssi)
{
    if (len < sizeof(swarm_header_t)) {
        return;
    }
    
    swarm_header_t header;
    memcpy(&header, data, sizeof(swarm_header_t));
    
    // Only process FORMATION messages
    if (header.msg_type != MSG_TYPE_FORMATION) {
        return;
    }
    
    // Extract payload
    const uint8_t *payload = data + sizeof(swarm_header_t);
    int payload_len = len - sizeof(swarm_header_t);
    
    process_formation_message(src_mac, payload, payload_len);
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t formation_init(void)
{
    ESP_LOGI(TAG, "Initializing formation module...");
    
    // Create mutex
    if (formation_mutex == NULL) {
        formation_mutex = xSemaphoreCreateMutex();
        if (formation_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Clear state
    memset(&current_config, 0, sizeof(current_config));
    memset(&my_position, 0, sizeof(my_position));
    current_config.spacing_mm = FORMATION_DEFAULT_SPACING_MM;
    my_formation_index = -1;
    position_valid = false;
    
    // Clear pattern registry
    memset(pattern_registry, 0, sizeof(pattern_registry));
    registered_patterns = 0;
    
    // Register built-in patterns
    register_builtin_patterns();
    
    ESP_LOGI(TAG, "Formation module initialized (%d patterns registered)", registered_patterns);
    return ESP_OK;
}

esp_err_t formation_start(void)
{
    if (running) {
        ESP_LOGW(TAG, "Formation already running");
        return ESP_OK;
    }
    
    uint16_t my_node_id = node_config_get_node_id();
    if (my_node_id == NODE_ID_INVALID) {
        ESP_LOGW(TAG, "Cannot start formation - node not provisioned");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting formation protocol...");
    
    // Register receive handler
    swarm_transport_register_recv_cb(formation_recv_handler);
    
    running = true;
    
    ESP_LOGI(TAG, "Formation protocol started");
    return ESP_OK;
}

void formation_stop(void)
{
    if (!running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping formation protocol...");
    running = false;
    ESP_LOGI(TAG, "Formation protocol stopped");
}

esp_err_t formation_set_type(formation_type_t type)
{
    // Validate type
    if (type >= FORMATION_COUNT) {
        ESP_LOGE(TAG, "Invalid formation type: %d", type);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if pattern is registered
    if (type != FORMATION_NONE && find_pattern(type) == NULL) {
        ESP_LOGE(TAG, "Formation type %d not registered", type);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Only leader can change formation
    if (!leader_election_is_leader()) {
        ESP_LOGW(TAG, "Cannot set formation - not leader");
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(formation_mutex, portMAX_DELAY);
    
    // Update config
    current_config.type = type;
    current_config.node_count = membership_get_count() + 1;  // +1 for self
    current_config.config_seq++;
    
    ESP_LOGI(TAG, "Formation set: %s (nodes=%d)", 
             formation_type_to_string(type), current_config.node_count);
    
    // Recalculate my position
    recalculate_my_position();
    
    // Copy for broadcast
    formation_config_t config_copy = current_config;
    
    xSemaphoreGive(formation_mutex);
    
    // Broadcast to swarm
    send_formation_config();
    
    // Notify callback
    if (changed_callback) {
        changed_callback(&config_copy);
    }
    
    return ESP_OK;
}

esp_err_t formation_set_spacing(int32_t spacing_mm)
{
    if (spacing_mm < FORMATION_MIN_SPACING_MM || spacing_mm > FORMATION_MAX_SPACING_MM) {
        ESP_LOGE(TAG, "Invalid spacing: %ld mm (must be %d-%d)",
                 (long)spacing_mm, FORMATION_MIN_SPACING_MM, FORMATION_MAX_SPACING_MM);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Only leader can change spacing
    if (!leader_election_is_leader()) {
        ESP_LOGW(TAG, "Cannot set spacing - not leader");
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(formation_mutex, portMAX_DELAY);
    
    current_config.spacing_mm = spacing_mm;
    current_config.config_seq++;
    
    ESP_LOGI(TAG, "Spacing set: %ld mm", (long)spacing_mm);
    
    // Recalculate position if formation active
    if (current_config.type != FORMATION_NONE) {
        recalculate_my_position();
    }
    
    formation_config_t config_copy = current_config;
    
    xSemaphoreGive(formation_mutex);
    
    // Broadcast to swarm
    send_formation_config();
    
    if (changed_callback) {
        changed_callback(&config_copy);
    }
    
    return ESP_OK;
}

esp_err_t formation_move(int32_t x_mm, int32_t y_mm, int16_t heading_deg)
{
    // Only leader can move formation
    if (!leader_election_is_leader()) {
        ESP_LOGW(TAG, "Cannot move formation - not leader");
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(formation_mutex, portMAX_DELAY);
    
    current_config.center_x_mm = x_mm;
    current_config.center_y_mm = y_mm;
    current_config.center_heading_deg = heading_deg;
    current_config.config_seq++;
    
    ESP_LOGI(TAG, "Formation move: center=(%ld, %ld) mm, heading=%d°",
             (long)x_mm, (long)y_mm, heading_deg);
    
    formation_config_t config_copy = current_config;
    
    xSemaphoreGive(formation_mutex);
    
    // Broadcast to swarm
    send_formation_config();
    
    if (changed_callback) {
        changed_callback(&config_copy);
    }
    
    return ESP_OK;
}

esp_err_t formation_get_my_position(formation_position_t *pos)
{
    if (pos == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(formation_mutex, portMAX_DELAY);
    
    if (!position_valid) {
        xSemaphoreGive(formation_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    *pos = my_position;
    
    xSemaphoreGive(formation_mutex);
    return ESP_OK;
}

int formation_get_my_index(void)
{
    xSemaphoreTake(formation_mutex, portMAX_DELAY);
    int index = my_formation_index;
    xSemaphoreGive(formation_mutex);
    return index;
}

esp_err_t formation_get_config(formation_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(formation_mutex, portMAX_DELAY);
    *config = current_config;
    xSemaphoreGive(formation_mutex);
    
    return ESP_OK;
}

bool formation_is_active(void)
{
    xSemaphoreTake(formation_mutex, portMAX_DELAY);
    bool active = (current_config.type != FORMATION_NONE);
    xSemaphoreGive(formation_mutex);
    return active;
}

void formation_register_changed_cb(formation_changed_cb_t cb)
{
    changed_callback = cb;
}

void formation_register_position_cb(position_assigned_cb_t cb)
{
    position_callback = cb;
}

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

static int cmd_formation(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: formation <type>\n");
        printf("Types: none, line, wedge, circle, column\n");
        return 1;
    }
    
    formation_type_t type = formation_type_from_string(argv[1]);
    if (type == FORMATION_NONE && strcmp(argv[1], "none") != 0) {
        printf("Unknown formation type: %s\n", argv[1]);
        printf("Available: none, line, wedge, circle, column\n");
        return 1;
    }
    
    esp_err_t err = formation_set_type(type);
    if (err == ESP_OK) {
        printf("Formation set to: %s\n", formation_type_to_string(type));
    } else if (err == ESP_ERR_INVALID_STATE) {
        printf("Error: Only leader can set formation\n");
    } else {
        printf("Error: %s\n", esp_err_to_name(err));
    }
    
    return 0;
}

static int cmd_spacing(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: spacing <mm>\n");
        printf("Range: %d - %d mm\n", FORMATION_MIN_SPACING_MM, FORMATION_MAX_SPACING_MM);
        return 1;
    }
    
    int32_t spacing = atoi(argv[1]);
    
    esp_err_t err = formation_set_spacing(spacing);
    if (err == ESP_OK) {
        printf("Spacing set to: %ld mm\n", (long)spacing);
    } else if (err == ESP_ERR_INVALID_STATE) {
        printf("Error: Only leader can set spacing\n");
    } else if (err == ESP_ERR_INVALID_ARG) {
        printf("Error: Spacing must be %d - %d mm\n", 
               FORMATION_MIN_SPACING_MM, FORMATION_MAX_SPACING_MM);
    } else {
        printf("Error: %s\n", esp_err_to_name(err));
    }
    
    return 0;
}

static int cmd_fmove(int argc, char **argv)
{
    if (argc < 3) {
        printf("Usage: fmove <x_mm> <y_mm> [heading]\n");
        printf("  x_mm, y_mm: Formation center position\n");
        printf("  heading: Formation heading in degrees (optional)\n");
        return 1;
    }
    
    int32_t x = atoi(argv[1]);
    int32_t y = atoi(argv[2]);
    int16_t heading = (argc >= 4) ? atoi(argv[3]) : 0;
    
    esp_err_t err = formation_move(x, y, heading);
    if (err == ESP_OK) {
        printf("Formation moving to: (%ld, %ld) mm, heading %d°\n", 
               (long)x, (long)y, heading);
    } else if (err == ESP_ERR_INVALID_STATE) {
        printf("Error: Only leader can move formation\n");
    } else {
        printf("Error: %s\n", esp_err_to_name(err));
    }
    
    return 0;
}

static int cmd_fstatus(int argc, char **argv)
{
    formation_print_status();
    return 0;
}

void formation_register_commands(void)
{
    const esp_console_cmd_t formation_cmd = {
        .command = "formation",
        .help = "Set formation type (leader only): none, line, wedge, circle, column",
        .hint = NULL,
        .func = &cmd_formation,
    };
    esp_console_cmd_register(&formation_cmd);
    
    const esp_console_cmd_t spacing_cmd = {
        .command = "spacing",
        .help = "Set node spacing in mm (leader only)",
        .hint = NULL,
        .func = &cmd_spacing,
    };
    esp_console_cmd_register(&spacing_cmd);
    
    const esp_console_cmd_t fmove_cmd = {
        .command = "fmove",
        .help = "Move formation to position (leader only)",
        .hint = NULL,
        .func = &cmd_fmove,
    };
    esp_console_cmd_register(&fmove_cmd);
    
    const esp_console_cmd_t fstatus_cmd = {
        .command = "fstatus",
        .help = "Show formation status",
        .hint = NULL,
        .func = &cmd_fstatus,
    };
    esp_console_cmd_register(&fstatus_cmd);
}

void formation_print_status(void)
{
    formation_config_t config;
    formation_get_config(&config);
    
    printf("\n=== Formation Status ===\n");
    printf("Type: %s\n", formation_type_to_string(config.type));
    printf("Active: %s\n", formation_is_active() ? "YES" : "NO");
    printf("Node count: %d\n", config.node_count);
    printf("Spacing: %ld mm\n", (long)config.spacing_mm);
    printf("Center: (%ld, %ld) mm\n", (long)config.center_x_mm, (long)config.center_y_mm);
    printf("Heading: %d°\n", config.center_heading_deg);
    printf("Config seq: %lu\n", (unsigned long)config.config_seq);
    
    printf("\n--- My Position ---\n");
    int my_index = formation_get_my_index();
    if (my_index >= 0) {
        formation_position_t pos;
        if (formation_get_my_position(&pos) == ESP_OK) {
            printf("Formation index: %d%s\n", my_index, my_index == 0 ? " (leader)" : "");
            printf("Offset: (%ld, %ld) mm\n", (long)pos.x_mm, (long)pos.y_mm);
            printf("Heading offset: %d°\n", pos.heading_deg);
            
            // Calculate absolute position
            int32_t abs_x = config.center_x_mm + pos.x_mm;
            int32_t abs_y = config.center_y_mm + pos.y_mm;
            int16_t abs_heading = config.center_heading_deg + pos.heading_deg;
            printf("Absolute position: (%ld, %ld) mm, heading %d°\n",
                   (long)abs_x, (long)abs_y, abs_heading);
        }
    } else {
        printf("Formation index: NOT ASSIGNED\n");
    }
    
    printf("\n--- Registered Patterns ---\n");
    for (int i = 0; i < registered_patterns; i++) {
        if (pattern_registry[i].registered) {
            printf("  %s (type=%d)\n", pattern_registry[i].name, pattern_registry[i].type);
        }
    }
    printf("\n");
}
