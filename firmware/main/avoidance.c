/**
 * avoidance.c
 *
 * Collision avoidance policy engine for the AQUABOTS swarm framework.
 *
 * The engine runs a FreeRTOS task that polls the obstacle_sense fusion layer
 * every AVOIDANCE_CHECK_INTERVAL_MS milliseconds. It tracks state transitions
 * through CLEAR → WARNING → DANGER and takes the following actions:
 *
 *   WARNING  — Logs the approaching obstacle and fires the state callback.
 *              No command is issued; the operator may respond manually.
 *
 *   DANGER   — Fires the state callback. If auto_stop is enabled and this
 *              node is the swarm leader, issues a CMD_STOP to the entire
 *              swarm. Simultaneously evaluates AVOIDANCE_ALT_BEARING_COUNT
 *              candidate bearings for a clear alternate heading and fires
 *              the suggest callback with the first clear option found.
 *
 * A state transition back to CLEAR automatically re-evaluates on the next
 * cycle; no manual reset is required.
 */

#include "avoidance.h"
#include "obstacle_sense.h"
#include "command_engine.h"
#include "leader_election.h"
#include "esp_log.h"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <limits.h>

static const char *TAG = "AVOIDANCE";

// ============================================================================
// INTERNAL STATE
// ============================================================================

static bool initialized                     = false;
static bool running                         = false;
static avoidance_state_t current_state      = AVOIDANCE_STATE_DISABLED;
static avoidance_config_t config;
static TaskHandle_t task_handle             = NULL;

static avoidance_state_cb_t    state_cb    = NULL;
static avoidance_suggest_cb_t  suggest_cb  = NULL;

// ============================================================================
// HELPERS
// ============================================================================

/**
 * Transition to a new state and notify the registered callback.
 * Suppresses no-op transitions (same → same).
 */
static void transition(avoidance_state_t new_state, const obstacle_detection_t *closest)
{
    if (new_state == current_state) {
        return;
    }

    ESP_LOGI(TAG, "State: %s → %s",
             avoidance_state_to_string(current_state),
             avoidance_state_to_string(new_state));

    current_state = new_state;

    if (state_cb) {
        state_cb(new_state, closest);
    }
}

/**
 * Search candidate bearings for the first one that is clear of obstacles
 * within the configured warning range and corridor width.
 *
 * Candidates are evaluated in a pattern that prioritises forward and
 * shallow turns before large heading changes: 0°, ±45°, ±90°, ±135°, 180°.
 *
 * @return Clear bearing in degrees, or INT16_MIN if none found.
 */
static int16_t find_clear_bearing(void)
{
    static const int16_t candidates[AVOIDANCE_ALT_BEARING_COUNT] = {
        0, 45, -45, 90, -90, 135, -135, 180
    };

    for (int i = 0; i < AVOIDANCE_ALT_BEARING_COUNT; i++) {
        if (obstacle_sense_path_clear(candidates[i],
                                      config.corridor_deg,
                                      config.warning_mm)) {
            return candidates[i];
        }
    }
    return INT16_MIN;
}

// ============================================================================
// AVOIDANCE TASK
// ============================================================================

static void avoidance_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Avoidance task started (interval=%d ms, danger=%d mm, warning=%d mm)",
             AVOIDANCE_CHECK_INTERVAL_MS, config.danger_mm, config.warning_mm);

    while (running) {
        vTaskDelay(pdMS_TO_TICKS(AVOIDANCE_CHECK_INTERVAL_MS));

        if (!config.enabled) {
            if (current_state != AVOIDANCE_STATE_DISABLED) {
                transition(AVOIDANCE_STATE_DISABLED, NULL);
            }
            continue;
        }

        /* Query closest obstacle across local and all remote swarm scans. */
        obstacle_detection_t closest;
        uint16_t closest_node;
        esp_err_t err = obstacle_sense_get_closest(&closest, &closest_node);

        if (err != ESP_OK) {
            /* No obstacles at all — transition to clear. */
            transition(AVOIDANCE_STATE_CLEAR, NULL);
            continue;
        }

        if (closest.range_mm <= config.danger_mm) {
            /* ---- DANGER ---- */
            ESP_LOGW(TAG, "DANGER: %d mm @ %d° (node %u)",
                     closest.range_mm, closest.bearing_deg, closest_node);

            transition(AVOIDANCE_STATE_DANGER, &closest);

            /* Issue swarm-wide stop if enabled and we are the leader. */
            if (config.auto_stop && leader_election_is_leader()) {
                esp_err_t stop_err = command_engine_send_command(CMD_STOP, 0, 0, 0, 0);
                if (stop_err == ESP_OK) {
                    ESP_LOGW(TAG, "Auto-stop issued to swarm (obstacle < %d mm)",
                             config.danger_mm);
                }
            }

            /* Search for a clear alternate heading and suggest it. */
            int16_t alt = find_clear_bearing();
            if (alt != INT16_MIN) {
                ESP_LOGI(TAG, "Suggested alternate bearing: %d°", alt);
                if (suggest_cb) {
                    suggest_cb(alt, config.warning_mm);
                }
            } else {
                ESP_LOGW(TAG, "No clear bearing found — all paths blocked");
            }

        } else if (closest.range_mm <= config.warning_mm) {
            /* ---- WARNING ---- */
            if (current_state != AVOIDANCE_STATE_WARNING) {
                ESP_LOGI(TAG, "WARNING: obstacle approaching — %d mm @ %d° (node %u)",
                         closest.range_mm, closest.bearing_deg, closest_node);
            }
            transition(AVOIDANCE_STATE_WARNING, &closest);

        } else {
            /* ---- CLEAR ---- */
            transition(AVOIDANCE_STATE_CLEAR, NULL);
        }
    }

    ESP_LOGI(TAG, "Avoidance task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t avoidance_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    config.enabled       = true;
    config.danger_mm     = AVOIDANCE_DANGER_MM_DEFAULT;
    config.warning_mm    = AVOIDANCE_WARNING_MM_DEFAULT;
    config.corridor_deg  = AVOIDANCE_CORRIDOR_DEG_DEFAULT;
    config.auto_stop     = true;
    current_state        = AVOIDANCE_STATE_DISABLED;

    initialized = true;

    ESP_LOGI(TAG, "Avoidance initialized — danger=%d mm, warning=%d mm, corridor=±%d°",
             config.danger_mm, config.warning_mm, config.corridor_deg / 2);

    return ESP_OK;
}

esp_err_t avoidance_start(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Call avoidance_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (running) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    running       = true;
    current_state = AVOIDANCE_STATE_CLEAR;

    BaseType_t ret = xTaskCreate(avoidance_task, "avoidance",
                                 4096, NULL, 3, &task_handle);
    if (ret != pdPASS) {
        running = false;
        ESP_LOGE(TAG, "Failed to create avoidance task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Avoidance started");
    return ESP_OK;
}

void avoidance_stop(void)
{
    if (!running) {
        return;
    }

    running       = false;
    task_handle   = NULL;
    current_state = AVOIDANCE_STATE_DISABLED;

    ESP_LOGI(TAG, "Avoidance stopped");
}

void avoidance_get_config(avoidance_config_t *out)
{
    if (out) {
        *out = config;
    }
}

void avoidance_set_config(const avoidance_config_t *in)
{
    if (!in) {
        return;
    }

    config = *in;

    ESP_LOGI(TAG, "Config updated — danger=%d mm, warning=%d mm, corridor=±%d°, auto_stop=%s",
             config.danger_mm, config.warning_mm,
             config.corridor_deg / 2,
             config.auto_stop ? "YES" : "NO");
}

avoidance_state_t avoidance_get_state(void)
{
    return current_state;
}

const char *avoidance_state_to_string(avoidance_state_t state)
{
    switch (state) {
        case AVOIDANCE_STATE_DISABLED: return "DISABLED";
        case AVOIDANCE_STATE_CLEAR:    return "CLEAR";
        case AVOIDANCE_STATE_WARNING:  return "WARNING";
        case AVOIDANCE_STATE_DANGER:   return "DANGER";
        default:                       return "UNKNOWN";
    }
}

bool avoidance_is_path_safe(int16_t bearing_deg, int16_t range_mm)
{
    return obstacle_sense_path_clear(bearing_deg, config.corridor_deg, range_mm);
}

void avoidance_register_state_cb(avoidance_state_cb_t cb)
{
    state_cb = cb;
}

void avoidance_register_suggest_cb(avoidance_suggest_cb_t cb)
{
    suggest_cb = cb;
}

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

static int cmd_avoid_status(int argc, char **argv)
{
    avoidance_print_status();
    return 0;
}

static int cmd_avoid_enable(int argc, char **argv)
{
    config.enabled = true;
    printf("Avoidance enabled\n");
    return 0;
}

static int cmd_avoid_disable(int argc, char **argv)
{
    config.enabled = false;
    printf("Avoidance disabled\n");
    return 0;
}

static int cmd_avoid_config(int argc, char **argv)
{
    if (argc < 3) {
        printf("Usage: avoid_config <param> <value>\n\n");
        printf("Parameters:\n");
        printf("  danger_mm     <mm>   Danger zone radius (default %d mm)\n",
               AVOIDANCE_DANGER_MM_DEFAULT);
        printf("  warning_mm    <mm>   Warning zone radius (default %d mm)\n",
               AVOIDANCE_WARNING_MM_DEFAULT);
        printf("  corridor_deg  <deg>  Path corridor total width (default %d°)\n",
               AVOIDANCE_CORRIDOR_DEG_DEFAULT);
        printf("  auto_stop     <0|1>  Issue STOP when danger triggered\n");
        printf("\nExamples:\n");
        printf("  avoid_config danger_mm 400\n");
        printf("  avoid_config auto_stop 0\n");
        return 1;
    }

    const char *param = argv[1];
    int value = atoi(argv[2]);

    if (strcmp(param, "danger_mm") == 0) {
        if (value < 30 || value > 4500) {
            printf("danger_mm must be between 30 and 4500\n");
            return 1;
        }
        config.danger_mm = (int16_t)value;
        printf("Danger zone set to %d mm\n", config.danger_mm);

    } else if (strcmp(param, "warning_mm") == 0) {
        if (value < 30 || value > 4500) {
            printf("warning_mm must be between 30 and 4500\n");
            return 1;
        }
        config.warning_mm = (int16_t)value;
        printf("Warning zone set to %d mm\n", config.warning_mm);

    } else if (strcmp(param, "corridor_deg") == 0) {
        if (value < 10 || value > 180) {
            printf("corridor_deg must be between 10 and 180\n");
            return 1;
        }
        config.corridor_deg = (int16_t)value;
        printf("Corridor width set to %d° (±%d°)\n", config.corridor_deg, config.corridor_deg / 2);

    } else if (strcmp(param, "auto_stop") == 0) {
        config.auto_stop = (value != 0);
        printf("Auto-stop %s\n", config.auto_stop ? "enabled" : "disabled");

    } else {
        printf("Unknown parameter: %s\n", param);
        printf("Valid parameters: danger_mm, warning_mm, corridor_deg, auto_stop\n");
        return 1;
    }

    return 0;
}

void avoidance_register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {
            .command = "avoid_status",
            .help    = "Show avoidance state and configuration",
            .hint    = NULL,
            .func    = &cmd_avoid_status,
        },
        {
            .command = "avoid_enable",
            .help    = "Enable collision avoidance",
            .hint    = NULL,
            .func    = &cmd_avoid_enable,
        },
        {
            .command = "avoid_disable",
            .help    = "Disable collision avoidance",
            .hint    = NULL,
            .func    = &cmd_avoid_disable,
        },
        {
            .command = "avoid_config",
            .help    = "Tune avoidance: avoid_config <param> <value>",
            .hint    = NULL,
            .func    = &cmd_avoid_config,
        },
    };

    for (int i = 0; i < (int)(sizeof(cmds) / sizeof(cmds[0])); i++) {
        esp_console_cmd_register(&cmds[i]);
    }
}

void avoidance_print_status(void)
{
    printf("\n=== Collision Avoidance Status ===\n");
    printf("State:          %s\n", avoidance_state_to_string(current_state));
    printf("Enabled:        %s\n", config.enabled ? "YES" : "NO");
    printf("Auto-stop:      %s\n", config.auto_stop ? "YES (leader only)" : "NO");
    printf("Danger zone:    < %d mm  → STOP issued\n", config.danger_mm);
    printf("Warning zone:   < %d mm  → alert only\n", config.warning_mm);
    printf("Corridor width: %d° (±%d° around checked bearing)\n",
           config.corridor_deg, config.corridor_deg / 2);

    printf("\nCurrent obstacle picture:\n");
    obstacle_detection_t closest;
    uint16_t closest_node;
    if (obstacle_sense_get_closest(&closest, &closest_node) == ESP_OK) {
        printf("  Closest: %d mm @ %d° from node %u\n",
               closest.range_mm, closest.bearing_deg, closest_node);

        /* Show path check for forward direction. */
        bool fwd_clear = obstacle_sense_path_clear(0, config.corridor_deg,
                                                    config.warning_mm);
        printf("  Forward path (0°, ±%d°, %d mm): %s\n",
               config.corridor_deg / 2, config.warning_mm,
               fwd_clear ? "CLEAR" : "BLOCKED");
    } else {
        printf("  No obstacles detected\n");
    }
    printf("\n");
}
