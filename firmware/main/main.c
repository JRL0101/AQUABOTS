/**
 * main.c
 *
 * AQUABOTS — ESP32-S3 Swarm Framework for Autonomous Aquatic Drones
 *
 * Entry point for the swarm coordination firmware. Initialises all modules,
 * registers callbacks, starts swarm protocols, and launches the serial console.
 *
 * Module startup order:
 *   1. NVS flash
 *   2. node_config   — loads or provisions persistent node identity
 *   3. swarm_transport — initialises ESP-NOW radio
 *   4. membership    — peer discovery via HELLO broadcasts
 *   5. leader_election — highest node_id election + TDMA beacons
 *   6. scheduler     — TDMA slot management
 *   7. command_engine — GOTO/HOLD/STOP/RESUME distribution
 *   8. formation     — coordinated spatial formation control
 *   9. obstacle_sense — sensor abstraction and swarm-wide scan fusion
 *  10. avoidance     — collision avoidance policy engine
 *
 * If a real A02YYUW ultrasonic sensor is connected, call a02yyuw_init()
 * here before obstacle_sense_start() and register the hardware driver:
 *
 *   #include "a02yyuw.h"
 *   a02yyuw_init(0, UART_NUM_1, GPIO_NUM_17, -1, 0);   // Forward sensor
 *   obstacle_sense_register_driver(a02yyuw_scan_driver);
 *
 * Without that call the system defaults to mock mode, which is fully
 * functional for swarm testing without physical sensors attached.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_console.h"
#include <stdlib.h>

#include "node_config.h"
#include "swarm_transport.h"
#include "membership.h"
#include "leader_election.h"
#include "scheduler.h"
#include "command_engine.h"
#include "formation.h"
#include "obstacle_sense.h"
#include "avoidance.h"
/* Uncomment to use real A02YYUW hardware sensor: */
/* #include "a02yyuw.h" */

static const char *TAG = "MAIN";

// ============================================================================
// MEMBERSHIP CALLBACKS
// ============================================================================

static void on_member_joined(const member_info_t *member)
{
    ESP_LOGI(TAG, "*** NEW MEMBER DISCOVERED ***");
    ESP_LOGI(TAG, "    Node ID: %u", member->node_id);
    ESP_LOGI(TAG, "    MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             member->mac[0], member->mac[1], member->mac[2],
             member->mac[3], member->mac[4], member->mac[5]);
    ESP_LOGI(TAG, "    RSSI: %d dBm", member->last_rssi);
}

static void on_member_lost(const member_info_t *member)
{
    ESP_LOGW(TAG, "*** MEMBER LOST (TIMEOUT) ***");
    ESP_LOGW(TAG, "    Node ID: %u", member->node_id);
}

// ============================================================================
// LEADER ELECTION CALLBACKS
// ============================================================================

static void on_leader_changed(uint16_t new_leader_id, bool is_self)
{
    if (is_self) {
        ESP_LOGI(TAG, "*** I AM NOW THE LEADER ***");
        ESP_LOGI(TAG, "    Node ID: %u", new_leader_id);
        ESP_LOGI(TAG, "    Generating TDMA schedule...");
    } else {
        ESP_LOGI(TAG, "*** NEW LEADER ELECTED ***");
        ESP_LOGI(TAG, "    Leader Node ID: %u", new_leader_id);
    }
}

static void on_schedule_updated(const tdma_slot_t *schedule, int num_slots)
{
    ESP_LOGI(TAG, "*** TDMA SCHEDULE UPDATED ***");
    ESP_LOGI(TAG, "    Number of slots: %d", num_slots);

    int my_slot = leader_election_get_my_slot();
    if (my_slot >= 0) {
        ESP_LOGI(TAG, "    My slot: %d (at %d ms)", my_slot, my_slot * 50);
    } else {
        ESP_LOGW(TAG, "    My slot: NOT ASSIGNED");
    }
}

// ============================================================================
// COMMAND ENGINE CALLBACKS
// ============================================================================

static bool on_command_received(const swarm_command_t *command)
{
    ESP_LOGI(TAG, "*** COMMAND RECEIVED ***");
    ESP_LOGI(TAG, "    Type: %s", command_engine_get_type_name(command->cmd_type));
    ESP_LOGI(TAG, "    ID: %lu", (unsigned long)command->cmd_id);
    ESP_LOGI(TAG, "    Target: %u", command->target_node);

    if (command->cmd_type == CMD_GOTO) {
        ESP_LOGI(TAG, "    Waypoint: (%ld, %ld)", (long)command->param1, (long)command->param2);
        ESP_LOGI(TAG, "    Heading: %ld°", (long)command->param3);
    }

    return true;
}

static void on_command_status(const swarm_command_t *command, command_status_t status)
{
    ESP_LOGI(TAG, "*** COMMAND STATUS CHANGE ***");
    ESP_LOGI(TAG, "    Command: %s", command_engine_get_type_name(command->cmd_type));
    ESP_LOGI(TAG, "    Status: %s", command_engine_get_status_name(status));
}

// ============================================================================
// FORMATION CALLBACKS
// ============================================================================

static void on_formation_changed(const formation_config_t *config)
{
    ESP_LOGI(TAG, "*** FORMATION CHANGED ***");
    ESP_LOGI(TAG, "    Type: %s", formation_type_to_string(config->type));
    ESP_LOGI(TAG, "    Nodes: %d", config->node_count);
    ESP_LOGI(TAG, "    Spacing: %ld mm", (long)config->spacing_mm);
    ESP_LOGI(TAG, "    Center: (%ld, %ld) mm",
             (long)config->center_x_mm, (long)config->center_y_mm);
}

static void on_position_assigned(const formation_position_t *position, int node_index)
{
    ESP_LOGI(TAG, "*** POSITION ASSIGNED ***");
    ESP_LOGI(TAG, "    Formation index: %d%s",
             node_index, node_index == 0 ? " (leader)" : "");
    ESP_LOGI(TAG, "    Offset: (%ld, %ld) mm",
             (long)position->x_mm, (long)position->y_mm);
    ESP_LOGI(TAG, "    Heading offset: %d°", position->heading_deg);
}

// ============================================================================
// OBSTACLE SENSING CALLBACKS
// ============================================================================

static void on_obstacle_scan(const obstacle_scan_t *scan)
{
    if (scan->num_detections > 0) {
        ESP_LOGI(TAG, "*** LOCAL SCAN: %d obstacle(s) ***", scan->num_detections);
        for (int i = 0; i < scan->num_detections && i < 3; i++) {
            ESP_LOGI(TAG, "    [%d] %d mm @ %d°",
                     i, scan->detections[i].range_mm, scan->detections[i].bearing_deg);
        }
    }
}

static void on_remote_obstacle(uint16_t node_id, const obstacle_scan_t *scan)
{
    if (scan->num_detections > 0) {
        ESP_LOGI(TAG, "*** REMOTE SCAN from node %u: %d obstacle(s) ***",
                 node_id, scan->num_detections);
    }
}

// ============================================================================
// AVOIDANCE CALLBACKS
// ============================================================================

static void on_avoidance_state_changed(avoidance_state_t new_state,
                                       const obstacle_detection_t *closest)
{
    switch (new_state) {
        case AVOIDANCE_STATE_CLEAR:
            ESP_LOGI(TAG, "*** AVOIDANCE: path clear ***");
            break;
        case AVOIDANCE_STATE_WARNING:
            if (closest) {
                ESP_LOGW(TAG, "*** AVOIDANCE WARNING: obstacle at %d mm @ %d° ***",
                         closest->range_mm, closest->bearing_deg);
            }
            break;
        case AVOIDANCE_STATE_DANGER:
            if (closest) {
                ESP_LOGE(TAG, "*** AVOIDANCE DANGER: obstacle at %d mm @ %d° — taking action ***",
                         closest->range_mm, closest->bearing_deg);
            }
            break;
        case AVOIDANCE_STATE_DISABLED:
            break;
    }
}

static void on_avoidance_suggest(int16_t bearing_deg, int16_t range_clear_mm)
{
    ESP_LOGI(TAG, "*** AVOIDANCE: suggested alternate bearing %d° (clear to %d mm) ***",
             bearing_deg, range_clear_mm);
}

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

static int cmd_members(int argc, char **argv)
{
    membership_print_table();
    return 0;
}

static int cmd_leader(int argc, char **argv)
{
    uint16_t leader_id = leader_election_get_leader();
    bool is_leader     = leader_election_is_leader();

    printf("\n=== Leader Information ===\n");
    if (leader_id == 0) {
        printf("Leader: NONE (election pending)\n");
    } else {
        printf("Leader Node ID: %u\n", leader_id);
        printf("I am leader: %s\n", is_leader ? "YES" : "NO");
    }

    leader_state_t state = leader_election_get_state();
    const char *state_str;
    switch (state) {
        case LEADER_STATE_FOLLOWER:  state_str = "FOLLOWER";  break;
        case LEADER_STATE_CANDIDATE: state_str = "CANDIDATE"; break;
        case LEADER_STATE_LEADER:    state_str = "LEADER";    break;
        default:                     state_str = "UNKNOWN";   break;
    }
    printf("State: %s\n\n", state_str);
    return 0;
}

static int cmd_schedule(int argc, char **argv)
{
    tdma_slot_t schedule[TDMA_MAX_SLOTS];
    int num_slots = leader_election_get_schedule(schedule, TDMA_MAX_SLOTS);

    printf("\n=== TDMA Schedule (%d slots) ===\n", num_slots);
    if (num_slots == 0) {
        printf("No schedule available yet.\n\n");
        return 0;
    }

    printf("%-8s %-10s %-15s\n", "Slot", "Node ID", "Time (ms)");
    printf("--------------------------------------------\n");

    int my_slot = leader_election_get_my_slot();
    for (int i = 0; i < num_slots; i++) {
        if (schedule[i].active) {
            const char *marker = (i == my_slot) ? " <-- ME" : "";
            printf("%-8d %-10u %-15d%s\n",
                   i, schedule[i].node_id, i * TDMA_SLOT_DURATION_MS, marker);
        }
    }

    printf("\n");
    if (my_slot >= 0) {
        printf("My slot: %d  |  Time to slot: %d ms\n\n",
               my_slot, leader_election_get_time_to_slot());
    } else {
        printf("My slot: NOT ASSIGNED\n\n");
    }
    return 0;
}

static int cmd_goto(int argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        printf("Usage: goto <x> <y> [heading]\n");
        printf("  x, y: target position (metres)\n");
        printf("  heading: target heading in degrees (default 0)\n");
        return 1;
    }

    int32_t x       = atoi(argv[1]);
    int32_t y       = atoi(argv[2]);
    int32_t heading = (argc >= 4) ? atoi(argv[3]) : 0;

    printf("Sending GOTO (%ld, %ld) heading %ld°\n",
           (long)x, (long)y, (long)heading);

    esp_err_t err = command_engine_send_command(CMD_GOTO, 0, x, y, heading);
    if (err == ESP_OK) {
        printf("Command sent to swarm\n");
    } else {
        printf("Failed: %s\n", esp_err_to_name(err));
        if (err == ESP_ERR_INVALID_STATE) {
            printf("(Only the leader can send commands)\n");
        }
    }
    return 0;
}

static int cmd_hold(int argc, char **argv)
{
    esp_err_t err = command_engine_send_command(CMD_HOLD, 0, 0, 0, 0);
    printf("HOLD: %s\n", err == ESP_OK ? "sent" : esp_err_to_name(err));
    if (err == ESP_ERR_INVALID_STATE) {
        printf("(Only the leader can send commands)\n");
    }
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    esp_err_t err = command_engine_send_command(CMD_STOP, 0, 0, 0, 0);
    printf("STOP: %s\n", err == ESP_OK ? "sent" : esp_err_to_name(err));
    if (err == ESP_ERR_INVALID_STATE) {
        printf("(Only the leader can send commands)\n");
    }
    return 0;
}

static int cmd_resume(int argc, char **argv)
{
    esp_err_t err = command_engine_send_command(CMD_RESUME, 0, 0, 0, 0);
    printf("RESUME: %s\n", err == ESP_OK ? "sent" : esp_err_to_name(err));
    if (err == ESP_ERR_INVALID_STATE) {
        printf("(Only the leader can send commands)\n");
    }
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    command_state_t state;

    printf("\n=== Command Status ===\n");
    if (!command_engine_get_state(&state)) {
        printf("Status: IDLE (no active command)\n\n");
        return 0;
    }

    printf("Command:  %s\n", command_engine_get_type_name(state.command.cmd_type));
    printf("Status:   %s\n", command_engine_get_status_name(state.status));
    printf("ID:       %lu\n", (unsigned long)state.command.cmd_id);

    if (state.command.cmd_type == CMD_GOTO) {
        printf("Target:   (%ld, %ld)\n",
               (long)state.command.param1, (long)state.command.param2);
        printf("Heading:  %ld°\n", (long)state.command.param3);
    }

    uint32_t elapsed = esp_log_timestamp() - state.start_time_ms;
    printf("Elapsed:  %lu ms\n", (unsigned long)elapsed);
    printf("\n");
    return 0;
}

static void register_console_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        { "members",  "Show discovered swarm members",           NULL, &cmd_members  },
        { "leader",   "Show leader information",                 NULL, &cmd_leader   },
        { "schedule", "Show TDMA schedule",                      NULL, &cmd_schedule },
        { "goto",     "Send GOTO command (leader only)",         NULL, &cmd_goto     },
        { "hold",     "Send HOLD command (leader only)",         NULL, &cmd_hold     },
        { "stop",     "Send STOP command (leader only)",         NULL, &cmd_stop     },
        { "resume",   "Send RESUME command (leader only)",       NULL, &cmd_resume   },
        { "status",   "Show current command status",             NULL, &cmd_status   },
    };

    for (int i = 0; i < (int)(sizeof(cmds) / sizeof(cmds[0])); i++) {
        esp_console_cmd_register(&cmds[i]);
    }

    formation_register_commands();
    obstacle_sense_register_commands();
    avoidance_register_commands();
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, " AQUABOTS — ESP32-S3 Swarm Framework");
    ESP_LOGI(TAG, "================================================");

    /* Initialize NVS (required by node_config and WiFi). */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Load or provision node identity. */
    node_config_init();

    uint16_t my_node_id = node_config_get_node_id();
    uint8_t  my_mac[6];
    node_config_get_mac(my_mac);

    ESP_LOGI(TAG, "------------------------------------------------");
    if (node_config_is_provisioned()) {
        ESP_LOGI(TAG, "Node ID : %u", my_node_id);
    } else {
        ESP_LOGI(TAG, "Node ID : NOT PROVISIONED");
        ESP_LOGI(TAG, "Use console: set_node_id <id>");
    }
    ESP_LOGI(TAG, "MAC     : %02x:%02x:%02x:%02x:%02x:%02x",
             my_mac[0], my_mac[1], my_mac[2],
             my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "------------------------------------------------");

    /* Bring up ESP-NOW radio. */
    swarm_transport_init();

    /* Initialise all swarm modules. */
    membership_init();
    leader_election_init();
    scheduler_init();
    command_engine_init();
    formation_init();
    obstacle_sense_init();
    avoidance_init();

    ESP_LOGI(TAG, "All modules initialised");

    /* Start serial provisioning console. */
    node_config_start_console();
    register_console_commands();

    if (!node_config_is_provisioned()) {
        ESP_LOGW(TAG, "=== NODE NOT PROVISIONED ===");
        ESP_LOGW(TAG, "Console only mode. Use 'set_node_id <id>' then restart.");
        return;
    }

    /* --- Optional: register real A02YYUW sensor here --- *
     * #include "a02yyuw.h"
     * a02yyuw_init(0, UART_NUM_1, GPIO_NUM_17, -1, 0);
     * obstacle_sense_register_driver(a02yyuw_scan_driver);
     * obstacle_sense_configure(4500, 120);
     * ---------------------------------------------------- */

    /* Register event callbacks. */
    membership_register_joined_cb(on_member_joined);
    membership_register_lost_cb(on_member_lost);
    leader_election_register_leader_changed_cb(on_leader_changed);
    leader_election_register_schedule_updated_cb(on_schedule_updated);
    command_engine_register_received_cb(on_command_received);
    command_engine_register_status_cb(on_command_status);
    formation_register_changed_cb(on_formation_changed);
    formation_register_position_cb(on_position_assigned);
    obstacle_sense_register_scan_cb(on_obstacle_scan);
    obstacle_sense_register_remote_cb(on_remote_obstacle);
    avoidance_register_state_cb(on_avoidance_state_changed);
    avoidance_register_suggest_cb(on_avoidance_suggest);

    /* Start all swarm protocols. */
    ESP_LOGI(TAG, "=== STARTING SWARM PROTOCOLS ===");
    membership_start();
    leader_election_start();
    command_engine_start();
    formation_start();
    obstacle_sense_start();
    avoidance_start();

    ESP_LOGI(TAG, "System running.");
    ESP_LOGI(TAG, "Console: help | info | members | leader | schedule |");
    ESP_LOGI(TAG, "         goto | hold | stop | resume | status |");
    ESP_LOGI(TAG, "         formation | spacing | fmove | fstatus |");
    ESP_LOGI(TAG, "         sensors | detections | mock_add | mock_clear | path_check |");
    ESP_LOGI(TAG, "         avoid_status | avoid_enable | avoid_disable | avoid_config");
}
