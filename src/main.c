#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "pn532.h"

// Pin definitions
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define LED_GPIO     2

// Authorized UID (change to your card's UID after first detection)
static const uint8_t AUTHORIZED_UID[] = {0xAA, 0xBB, 0xCC, 0xDD};
static const size_t AUTHORIZED_UID_LEN = 4;

static const char *TAG = "MAIN";

void app_main(void) {
    // Initialize LED GPIO
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
    
    // Initialize PN532
    ESP_LOGI(TAG, "Initializing PN532...");
    if (!pn532_init(PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS)) {
        ESP_LOGE(TAG, "Failed to initialize PN532");
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (!pn532_sam_config()) {
        ESP_LOGE(TAG, "Failed to configure PN532");
        return;
    }
    
    ESP_LOGI(TAG, "PN532 initialized successfully, waiting for NFC cards...");
    
    while (1) {
        uint8_t uid[10];
        uint8_t uid_len = 0;
        
        if (pn532_read_passive_target(uid, &uid_len)) {
            ESP_LOGI(TAG, "Card detected! UID:");
            ESP_LOG_BUFFER_HEX(TAG, uid, uid_len);
            
            // Check if UID matches authorized card
            if (uid_len == AUTHORIZED_UID_LEN && 
                memcmp(uid, AUTHORIZED_UID, AUTHORIZED_UID_LEN) == 0) {
                ESP_LOGI(TAG, "✓ Authorized card detected!");
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(3000));
                gpio_set_level(LED_GPIO, 0);
            } else {
                ESP_LOGW(TAG, "✗ Unauthorized card");
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
