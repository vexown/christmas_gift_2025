#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "pn532.h"
#include "driver/i2c.h"

/* Pin Mapping */
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define PN532_RST_PIN           4
#define MOTOR_DRIVER_LPWM_PIN   14
#define MOTOR_DRIVER_RPWM_PIN   13
#define MOTOR_DRIVER_L_EN_PIN   26
#define MOTOR_DRIVER_R_EN_PIN   25
#define ENDSTOP_CHECK_PIN       19
#define ENDSTOP_GND_PIN         5

/* I2C Configuration */
#define I2C_MASTER_FREQ_HZ   100000

/* RFID Configuration */
#define SCAN_INTERVAL_MS     500
#define CARD_REMOVE_DELAY_MS 2000

/* WiFi AP Configuration */
#define WIFI_AP_SSID         "ESP32-NFC-Door"
#define WIFI_AP_PASSWORD     "12345678"
#define WIFI_AP_CHANNEL      1
#define WIFI_AP_MAX_CONN     4

static const char *TAG = "NFC_APP";

// Door state
static bool door_open = false;

// Authorized UIDs (add your allowed cards here)
typedef struct {
    uint8_t uid[PN532_MAX_UID_LENGTH];
    uint8_t uid_len;
    const char *name;
} authorized_card_t;

static const authorized_card_t authorized_cards[] = {
    // Example UIDs - replace with your actual card UIDs
    { .uid = {0x04, 0x5A, 0x3B, 0x2A}, .uid_len = 4, .name = "Master Card" },
    { .uid = {0x12, 0x34, 0x56, 0x78}, .uid_len = 4, .name = "User Card 1" },
    { .uid = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00}, .uid_len = 7, .name = "User Card 2" },
    // Add more authorized cards here
};

#define NUM_AUTHORIZED_CARDS (sizeof(authorized_cards) / sizeof(authorized_card_t))

// Forward declarations
static esp_err_t wifi_init_softap(void);
static httpd_handle_t start_webserver(void);

// Door control functions
static void open_door(const char *reason) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          OPENING DOOR!               ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "Reason: %s", reason);
    
    door_open = true;
    
    // TODO: Add actual door control logic
    // - Activate relay/solenoid
    // - GPIO output HIGH
    
    ESP_LOGI(TAG, "Door is now OPEN");
    ESP_LOGI(TAG, "");
}

static void close_door(const char *reason) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          CLOSING DOOR!               ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "Reason: %s", reason);
    
    door_open = false;
    
    // TODO: Add actual door control logic
    // - Deactivate relay/solenoid
    // - GPIO output LOW
    
    ESP_LOGI(TAG, "Door is now CLOSED");
    ESP_LOGI(TAG, "");
}

// HTTP Handlers
static esp_err_t root_get_handler(httpd_req_t *req) {
    const char *html = 
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "  <meta charset='UTF-8'>"
        "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "  <title>ESP32 Door Control</title>"
        "  <style>"
        "    body {"
        "      font-family: Arial, sans-serif;"
        "      text-align: center;"
        "      margin: 50px;"
        "      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);"
        "      color: white;"
        "    }"
        "    h1 {"
        "      font-size: 2.5em;"
        "      margin-bottom: 20px;"
        "    }"
        "    .container {"
        "      background: rgba(255, 255, 255, 0.1);"
        "      border-radius: 20px;"
        "      padding: 40px;"
        "      max-width: 500px;"
        "      margin: 0 auto;"
        "      box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);"
        "    }"
        "    .status {"
        "      font-size: 1.8em;"
        "      margin: 30px 0;"
        "      padding: 20px;"
        "      border-radius: 10px;"
        "      background: rgba(255, 255, 255, 0.2);"
        "    }"
        "    .button {"
        "      font-size: 1.5em;"
        "      padding: 20px 40px;"
        "      margin: 15px;"
        "      border: none;"
        "      border-radius: 10px;"
        "      cursor: pointer;"
        "      color: white;"
        "      font-weight: bold;"
        "      transition: all 0.3s;"
        "      box-shadow: 0 4px 15px 0 rgba(0, 0, 0, 0.2);"
        "    }"
        "    .button:hover {"
        "      transform: translateY(-3px);"
        "      box-shadow: 0 6px 20px 0 rgba(0, 0, 0, 0.3);"
        "    }"
        "    .button:active {"
        "      transform: translateY(-1px);"
        "    }"
        "    .open-btn {"
        "      background: linear-gradient(135deg, #11998e 0%, #38ef7d 100%);"
        "    }"
        "    .close-btn {"
        "      background: linear-gradient(135deg, #eb3349 0%, #f45c43 100%);"
        "    }"
        "    .info {"
        "      margin-top: 30px;"
        "      font-size: 0.9em;"
        "      opacity: 0.8;"
        "    }"
        "  </style>"
        "</head>"
        "<body>"
        "  <div class='container'>"
        "    <h1>🚪 Door Control</h1>"
        "    <div class='status'>Door Status: <span id='status'>%s</span></div>"
        "    <form action='/open' method='get' style='display:inline'>"
        "      <button type='submit' class='button open-btn'>🔓 OPEN DOOR</button>"
        "    </form>"
        "    <br>"
        "    <form action='/close' method='get' style='display:inline'>"
        "      <button type='submit' class='button close-btn'>🔒 CLOSE DOOR</button>"
        "    </form>"
        "    <div class='info'>"
        "      <p>ESP32 NFC Access Control System</p>"
        "      <p>Tap an NFC card or use buttons above</p>"
        "    </div>"
        "  </div>"
        "</body>"
        "</html>";
    
    char response[2048];
    snprintf(response, sizeof(response), html, door_open ? "🟢 OPEN" : "🔴 CLOSED");
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t open_door_handler(httpd_req_t *req) {
    open_door("Web interface button");
    
    // Redirect back to main page
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t close_door_handler(httpd_req_t *req) {
    close_door("Web interface button");
    
    // Redirect back to main page
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Client connected, MAC: %02x:%02x:%02x:%02x:%02x:%02x, AID: %d",
                 event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Client disconnected, MAC: %02x:%02x:%02x:%02x:%02x:%02x, AID: %d",
                 event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
    }
}

// Initialize WiFi in AP mode
static esp_err_t wifi_init_softap(void) {
    ESP_LOGI(TAG, "Initializing WiFi AP...");
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      WiFi AP Started Successfully    ║");
    ESP_LOGI(TAG, "╠══════════════════════════════════════╣");
    ESP_LOGI(TAG, "║ SSID:     %-27s║", WIFI_AP_SSID);
    ESP_LOGI(TAG, "║ Password: %-27s║", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "║ IP:       192.168.4.1                ║");
    ESP_LOGI(TAG, "║ URL:      http://192.168.4.1         ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    return ESP_OK;
}

// Start HTTP server
static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // Root page
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);

        // Open door endpoint
        httpd_uri_t open = {
            .uri       = "/open",
            .method    = HTTP_GET,
            .handler   = open_door_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &open);

        // Close door endpoint
        httpd_uri_t close = {
            .uri       = "/close",
            .method    = HTTP_GET,
            .handler   = close_door_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &close);

        ESP_LOGI(TAG, "✓ HTTP server started successfully");
        return server;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    return NULL;
}

// Helper function to check if UID is authorized
static bool is_authorized(const uint8_t *uid, uint8_t uid_len, const char **card_name) {
    for (int i = 0; i < NUM_AUTHORIZED_CARDS; i++) {
        if (uid_len == authorized_cards[i].uid_len &&
            memcmp(uid, authorized_cards[i].uid, uid_len) == 0) {
            *card_name = authorized_cards[i].name;
            return true;
        }
    }
    return false;
}

// Helper function to print UID in a nice format
static void print_uid(const uint8_t *uid, uint8_t uid_len) {
    printf("Card UID: ");
    for (int i = 0; i < uid_len; i++) {
        printf("%02X", uid[i]);
        if (i < uid_len - 1) {
            printf(":");
        }
    }
    printf("\n");
}

// Helper function to print firmware info
static void print_firmware_info(uint8_t ic, uint8_t ver, uint8_t rev, uint8_t support) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════╗");
    ESP_LOGI(TAG, "║   PN532 Firmware Information   ║");
    ESP_LOGI(TAG, "╠════════════════════════════════╣");
    ESP_LOGI(TAG, "║ Chip:     PN5%02X                ║", ic);
    ESP_LOGI(TAG, "║ Version:  %d.%-2d                 ║", ver, rev);
    ESP_LOGI(TAG, "║ Support:  0x%02X                ║", support);
    ESP_LOGI(TAG, "╚════════════════════════════════╝");
    ESP_LOGI(TAG, "");
}

void app_main(void)
{
    esp_err_t ret;
    
    // Print banner
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   PN532 NFC Access Control System   ║");
    ESP_LOGI(TAG, "║   with WiFi Web Interface           ║");
    ESP_LOGI(TAG, "║   ESP-IDF v%s                  ║", esp_get_idf_version());
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Authorized cards: %d", NUM_AUTHORIZED_CARDS);
    ESP_LOGI(TAG, "");
    
    // Initialize NVS (required for WiFi)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize WiFi AP
    wifi_init_softap();
    
    // Start web server
    httpd_handle_t server = start_webserver();
    if (server == NULL) {
        ESP_LOGE(TAG, "Failed to start web server");
        return;
    }
    
    // Initialize PN532
    ret = pn532_init(I2C_NUM_0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 
                     PN532_RST_PIN, I2C_MASTER_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PN532: %s", esp_err_to_name(ret));
        return;
    }
    
    // Optional: Enable debug logging
    // pn532_set_debug(true);
    
    // Perform hardware reset
    ret = pn532_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset PN532: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Wake up the module
    ret = pn532_wakeup();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up PN532: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Test communication
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Testing PN532 Communication");
    ESP_LOGI(TAG, "========================================\n");
    
    // Get and display firmware version
    uint8_t ic, ver, rev, support;
    ret = pn532_get_firmware_version(&ic, &ver, &rev, &support);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ GetFirmwareVersion FAILED");
        ESP_LOGE(TAG, "\nTroubleshooting:");
        ESP_LOGE(TAG, "  1. Check DIP switches: [OFF][ON] for I2C mode");
        ESP_LOGE(TAG, "  2. Verify wiring connections");
        ESP_LOGE(TAG, "  3. Check 3.3V power supply");
        goto cleanup;
    }
    
    print_firmware_info(ic, ver, rev, support);
    
    // Configure SAM
    ret = pn532_sam_configuration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ SAMConfiguration FAILED");
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "\n╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ✓✓✓ ALL SYSTEMS READY! ✓✓✓       ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝\n");
    
    // Main application loop
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "System Ready - Waiting for NFC cards...");
    ESP_LOGI(TAG, "========================================\n");
    
    uint8_t uid[PN532_MAX_UID_LENGTH];
    uint8_t uid_len = 0;
    
    while (1) {
        // Try to detect a card
        ret = pn532_read_passive_target(uid, &uid_len);
        
        if (ret == ESP_OK) {
            // Card detected!
            ESP_LOGI(TAG, "\n╔══════════════════════════════════════╗");
            ESP_LOGI(TAG, "║          CARD DETECTED!              ║");
            ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
            
            // Print UID
            print_uid(uid, uid_len);
            
            // Check authorization
            const char *card_name;
            if (is_authorized(uid, uid_len, &card_name)) {
                // Authorized card - open door
                open_door(card_name);
            } else {
                // Unauthorized card
                ESP_LOGW(TAG, "");
                ESP_LOGW(TAG, "╔══════════════════════════════════════╗");
                ESP_LOGW(TAG, "║          ACCESS DENIED!              ║");
                ESP_LOGW(TAG, "╚══════════════════════════════════════╝");
                ESP_LOGW(TAG, "");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            
            ESP_LOGI(TAG, "Remove card to scan again...\n");
            vTaskDelay(pdMS_TO_TICKS(CARD_REMOVE_DELAY_MS));
            
        } else if (ret != ESP_ERR_NOT_FOUND && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Error reading card: %s", esp_err_to_name(ret));
        }
        
        vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
    
cleanup:
    pn532_deinit();
    if (server) {
        httpd_stop(server);
    }
    ESP_LOGE(TAG, "Application terminated");
}