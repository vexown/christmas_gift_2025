#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pn532.h"
#include "driver/i2c.h"

// Application Configuration
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_FREQ_HZ   100000    // 100kHz
#define PN532_RST_PIN        4

#define SCAN_INTERVAL_MS     500
#define CARD_REMOVE_DELAY_MS 2000

static const char *TAG = "NFC_APP";

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

// Dummy function to simulate door opening
static void open_door(const char *card_name) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          ACCESS GRANTED!             ║");
    ESP_LOGI(TAG, "║         Opening door...              ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "Card: %s", card_name);
    
    // TODO: Add your actual door control logic here
    // Examples:
    // - Activate relay/solenoid
    // - Send signal to door controller
    // - Log access event to database
    
    // Simulate door staying open for 5 seconds
    ESP_LOGI(TAG, "Door will remain open for 5 seconds...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Door closed.");
    ESP_LOGI(TAG, "");
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
    ESP_LOGI(TAG, "║   ESP-IDF v%s                  ║", esp_get_idf_version());
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Authorized cards: %d", NUM_AUTHORIZED_CARDS);
    ESP_LOGI(TAG, "");
    
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
    ESP_LOGI(TAG, "║   ✓✓✓ ALL TESTS PASSED! ✓✓✓        ║");
    ESP_LOGI(TAG, "║   PN532 is ready for NFC operations  ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝\n");
    
    // Main application loop - NFC card scanning
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "NFC Access Control System Ready");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Present an NFC card to check access...\n");
    
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
                // Unauthorized card - deny access
                ESP_LOGW(TAG, "");
                ESP_LOGW(TAG, "╔══════════════════════════════════════╗");
                ESP_LOGW(TAG, "║          ACCESS DENIED!              ║");
                ESP_LOGW(TAG, "║      Unauthorized card detected      ║");
                ESP_LOGW(TAG, "╚══════════════════════════════════════╝");
                ESP_LOGW(TAG, "");
                
                // TODO: Add your access denied logic here
                // Examples:
                // - Sound alarm/buzzer
                // - Log unauthorized access attempt
                // - Send notification
                
                // Wait a bit before scanning again
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            
            // Wait for card to be removed
            ESP_LOGI(TAG, "Remove card to scan again...\n");
            vTaskDelay(pdMS_TO_TICKS(CARD_REMOVE_DELAY_MS));
            
        } else if (ret == ESP_ERR_NOT_FOUND) {
            // No card present (expected during scanning)
            // Don't log anything to reduce spam
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Timeout waiting for card (also expected)
        } else {
            // Actual error
            ESP_LOGE(TAG, "Error reading card: %s", esp_err_to_name(ret));
        }
        
        // Scan at regular intervals
        vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
    
cleanup:
    pn532_deinit();
    ESP_LOGE(TAG, "Application terminated");
}