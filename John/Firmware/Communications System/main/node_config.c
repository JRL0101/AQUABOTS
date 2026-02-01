/**
 * node_config.c
 * 
 * Node identity management with NVS persistence.
 */

#include "node_config.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "linenoise/linenoise.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "NODE_CONFIG";

// NVS storage
#define NVS_NAMESPACE "swarm_config"
#define NVS_KEY_NODE_ID "node_id"

// Default node_id from menuconfig (fallback if not provisioned)
#ifndef CONFIG_DEFAULT_NODE_ID
#define CONFIG_DEFAULT_NODE_ID 0xFFFF  // Not provisioned by default
#endif

// Internal state
static uint16_t current_node_id = NODE_ID_INVALID;
static bool provisioned = false;

// ============================================================================
// NVS OPERATIONS
// ============================================================================

static esp_err_t load_node_id_from_nvs(uint16_t *node_id)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace not found, node not provisioned");
        return err;
    }
    
    err = nvs_get_u16(nvs_handle, NVS_KEY_NODE_ID, node_id);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded node_id=%u from NVS", *node_id);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "node_id not found in NVS");
    } else {
        ESP_LOGE(TAG, "Error reading node_id from NVS: %s", esp_err_to_name(err));
    }
    
    return err;
}

static esp_err_t save_node_id_to_nvs(uint16_t node_id)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_u16(nvs_handle, NVS_KEY_NODE_ID, node_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing node_id to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved node_id=%u to NVS", node_id);
    } else {
        ESP_LOGE(TAG, "Error committing to NVS: %s", esp_err_to_name(err));
    }
    
    return err;
}

// ============================================================================
// SERIAL CONSOLE COMMANDS
// ============================================================================

static int cmd_set_node_id(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: set_node_id <id>\n");
        printf("  id: Node ID (1-254)\n");
        return 1;
    }
    
    int node_id = atoi(argv[1]);
    
    if (node_id < NODE_ID_MIN || node_id > NODE_ID_MAX) {
        printf("Error: node_id must be between %d and %d\n", NODE_ID_MIN, NODE_ID_MAX);
        return 1;
    }
    
    esp_err_t err = node_config_set_node_id((uint16_t)node_id);
    if (err == ESP_OK) {
        printf("Node ID set to %d. Restart to apply.\n", node_id);
    } else {
        printf("Error setting node_id: %s\n", esp_err_to_name(err));
        return 1;
    }
    
    return 0;
}

static int cmd_get_node_id(int argc, char **argv)
{
    uint16_t node_id = node_config_get_node_id();
    
    if (node_id == NODE_ID_INVALID) {
        printf("Node is NOT provisioned (node_id = INVALID)\n");
        printf("Use 'set_node_id <id>' to provision this node.\n");
    } else {
        printf("Node ID: %u\n", node_id);
    }
    
    uint8_t mac[6];
    node_config_get_mac(mac);
    printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    return 0;
}

static int cmd_show_info(int argc, char **argv)
{
    printf("\n=== Node Information ===\n");
    
    uint16_t node_id = node_config_get_node_id();
    printf("Node ID: ");
    if (node_id == NODE_ID_INVALID) {
        printf("NOT PROVISIONED\n");
    } else {
        printf("%u\n", node_id);
    }
    
    uint8_t mac[6];
    node_config_get_mac(mac);
    printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    printf("Provisioned: %s\n", node_config_is_provisioned() ? "YES" : "NO");
    printf("\n");
    
    return 0;
}

static void register_console_commands(void)
{
    const esp_console_cmd_t set_node_id_cmd = {
        .command = "set_node_id",
        .help = "Set node ID (1-254) and save to NVS",
        .hint = NULL,
        .func = &cmd_set_node_id,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_node_id_cmd));
    
    const esp_console_cmd_t get_node_id_cmd = {
        .command = "get_node_id",
        .help = "Display current node ID and MAC",
        .hint = NULL,
        .func = &cmd_get_node_id,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_node_id_cmd));
    
    const esp_console_cmd_t info_cmd = {
        .command = "info",
        .help = "Show node information",
        .hint = NULL,
        .func = &cmd_show_info,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&info_cmd));
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t node_config_init(void)
{
    ESP_LOGI(TAG, "Initializing node configuration...");
    
    // Try to load from NVS first
    uint16_t stored_id;
    esp_err_t err = load_node_id_from_nvs(&stored_id);
    
    if (err == ESP_OK && stored_id >= NODE_ID_MIN && stored_id <= NODE_ID_MAX) {
        // Valid node_id found in NVS
        current_node_id = stored_id;
        provisioned = true;
        ESP_LOGI(TAG, "Node provisioned: node_id=%u", current_node_id);
    } else {
        // Use default from menuconfig
        current_node_id = CONFIG_DEFAULT_NODE_ID;
        provisioned = (current_node_id != NODE_ID_INVALID);
        
        if (provisioned) {
            ESP_LOGW(TAG, "Using default node_id=%u from menuconfig", current_node_id);
        } else {
            ESP_LOGW(TAG, "Node NOT provisioned. Use serial command 'set_node_id <id>' to provision.");
        }
    }
    
    return ESP_OK;
}

uint16_t node_config_get_node_id(void)
{
    return current_node_id;
}

esp_err_t node_config_set_node_id(uint16_t node_id)
{
    if (node_id < NODE_ID_MIN || node_id > NODE_ID_MAX) {
        ESP_LOGE(TAG, "Invalid node_id: %u (must be %u-%u)", 
                 node_id, NODE_ID_MIN, NODE_ID_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t err = save_node_id_to_nvs(node_id);
    if (err == ESP_OK) {
        current_node_id = node_id;
        provisioned = true;
        ESP_LOGI(TAG, "Node ID updated to %u", node_id);
    }
    
    return err;
}

bool node_config_is_provisioned(void)
{
    return provisioned;
}

void node_config_get_mac(uint8_t *mac)
{
    esp_wifi_get_mac(WIFI_IF_STA, mac);
}

void node_config_start_console(void)
{
    ESP_LOGI(TAG, "Starting serial console...");
    
    // Initialize console
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "swarm> ";
    repl_config.max_cmdline_length = 256;
    
    // Install console REPL environment
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    
    // Register commands
    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_console_commands();
    
    // Start console
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    
    ESP_LOGI(TAG, "Console started. Type 'help' for commands.");
}