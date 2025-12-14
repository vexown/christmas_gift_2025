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
    ESP_LOGI(TAG, "║   PN532 NFC Reader Application      ║");
    ESP_LOGI(TAG, "║   ESP-IDF v%s                  ║", esp_get_idf_version());
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
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
    ESP_LOGI(TAG, "Starting NFC Card Detection Loop");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Place an NFC card near the reader\n");
    
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
            printf("\n");
            
            // TODO: Add your card processing logic here
            // Examples:
            // - Store UID in database
            // - Grant/deny access
            // - Read/write NDEF messages
            // - Authenticate blocks
            
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