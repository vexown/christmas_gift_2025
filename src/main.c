#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// I2C Configuration
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_FREQ_HZ   100000    // 100kHz

#define PN532_RST_PIN        4
#define PN532_I2C_ADDRESS    0x24

// PN532 Protocol Defines
#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)
#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)

#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)

#define PN532_I2C_READY                     (0x01)

#define PN532_MIFARE_ISO14443A              (0x00)

static const char *TAG = "PN532";

// Enable this for detailed debug output
#define PN532_DEBUG_VERBOSE 0  // Set to 1 for detailed logs

#define PN532_TIMEOUT_MS 1000

// ACK frame
static const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

void pn532_reset_gpio_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PN532_RST_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

void pn532_reset_gpio(void) {
    ESP_LOGI(TAG, "Performing hardware reset...");
    gpio_set_level(PN532_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PN532_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500)); // Hold in reset
    gpio_set_level(PN532_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for chip to boot
    ESP_LOGI(TAG, "Reset complete, chip should be ready");
}

esp_err_t i2c_master_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PN532_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    return ret;
}

// Wakeup and check if ready
esp_err_t pn532_wakeup(void) {
    ESP_LOGI(TAG, "Waking up PN532...");
    
    // Send wakeup by attempting to read (PN532 wakes on I2C activity)
    uint8_t dummy[10];
    for (int i = 0; i < 3; i++) {
        esp_err_t ret = i2c_master_receive(dev_handle, dummy, sizeof(dummy), PN532_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Wakeup sequence complete");
    return ESP_OK;
}

// Check if PN532 is ready (improved version with detailed debug)
esp_err_t pn532_wait_ready(uint32_t timeout_ms) {
    uint32_t start = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    ESP_LOGI(TAG, "Waiting for PN532 ready status...");
    
    uint32_t poll_count = 0;
    
    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        uint8_t status;
        
        esp_err_t ret = i2c_master_receive(dev_handle, &status, 1, 100);
        
        poll_count++;
        
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "[Poll #%lu] Status byte: 0x%02X %s", 
                     poll_count, status, 
                     status == PN532_I2C_READY ? "(READY!)" : "(busy)");
            
            if (status == PN532_I2C_READY) {
                ESP_LOGI(TAG, "✓ PN532 is ready! (took %lu polls, %lu ms)", 
                         poll_count, 
                         (xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
                return ESP_OK;
            }
        } else {
            ESP_LOGD(TAG, "[Poll #%lu] I2C error: %s", poll_count, esp_err_to_name(ret));
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGE(TAG, "Timeout waiting for ready (polled %lu times)", poll_count);
    return ESP_ERR_TIMEOUT;
}

// Write command to PN532
esp_err_t pn532_write_command(const uint8_t *cmd, uint8_t cmd_len) {
    uint8_t checksum;
    uint8_t len = cmd_len + 1; // +1 for TFI byte
    
    // Calculate checksum
    checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
    checksum += len;
    checksum += (~len + 1);
    checksum += PN532_HOSTTOPN532;
    
    for (uint8_t i = 0; i < cmd_len; i++) {
        checksum += cmd[i];
    }
    checksum = ~checksum;
    
    // Build frame
    uint8_t frame[256];
    uint8_t idx = 0;
    
    frame[idx++] = PN532_PREAMBLE;
    frame[idx++] = PN532_STARTCODE1;
    frame[idx++] = PN532_STARTCODE2;
    frame[idx++] = len;
    frame[idx++] = ~len + 1;
    frame[idx++] = PN532_HOSTTOPN532;
    
    for (uint8_t i = 0; i < cmd_len; i++) {
        frame[idx++] = cmd[i];
    }
    
    frame[idx++] = checksum;
    frame[idx++] = PN532_POSTAMBLE;
    
    if (PN532_DEBUG_VERBOSE) {
        ESP_LOGI(TAG, "Sending command (0x%02X), %d bytes total:", cmd[0], idx);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, frame, idx, ESP_LOG_INFO);
    }
    
    esp_err_t ret = i2c_master_transmit(dev_handle, frame, idx, PN532_TIMEOUT_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// Read data from PN532 (including leading ready byte)
esp_err_t pn532_read_data(uint8_t *buffer, uint8_t len) {
    esp_err_t ret = i2c_master_receive(dev_handle, buffer, len, PN532_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        if (PN532_DEBUG_VERBOSE) {
            ESP_LOGI(TAG, "Read %d bytes:", len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, len, ESP_LOG_INFO);
        }
    } else {
        ESP_LOGE(TAG, "I2C receive failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// Read ACK frame
esp_err_t pn532_read_ack(void) {
    uint8_t ack_buf[7]; // +1 for status byte
    
    // Wait longer for PN532 to prepare ACK
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Wait for ready status
    uint32_t timeout = 100; // 100ms timeout
    uint32_t start = xTaskGetTickCount();
    bool ready = false;
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout)) {
        uint8_t status;
        esp_err_t ret = i2c_master_receive(dev_handle, &status, 1, 50);
        
        if (ret == ESP_OK && status == PN532_I2C_READY) {
            ready = true;
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    if (!ready) {
        ESP_LOGE(TAG, "PN532 not ready for ACK read");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t ret = pn532_read_data(ack_buf, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ACK");
        return ret;
    }
    
    // Skip first byte (status), compare rest
    if (memcmp(ack_buf + 1, PN532_ACK, 6) != 0) {
        ESP_LOGE(TAG, "Invalid ACK frame!");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✓ ACK received");
    return ESP_OK;
}

// Write command and check ACK
esp_err_t pn532_write_cmd_check_ack(const uint8_t *cmd, uint8_t len) {
    esp_err_t ret = pn532_write_command(cmd, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write command");
        return ret;
    }
    
    ret = pn532_read_ack();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No ACK received");
        return ret;
    }
    
    return ESP_OK;
}

// SAM Configuration
esp_err_t pn532_SAMConfiguration(void) {
    ESP_LOGI(TAG, "\n--- SAM Configuration ---");
    
    uint8_t cmd[] = {
        PN532_COMMAND_SAMCONFIGURATION,
        0x01,  // Normal mode
        0x14,  // Timeout
        0x00   // Don't use IRQ
    };
    
    if (pn532_write_cmd_check_ack(cmd, sizeof(cmd)) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Wait for response ready
    vTaskDelay(pdMS_TO_TICKS(100));
    if (pn532_wait_ready(500) != ESP_OK) {
        ESP_LOGE(TAG, "No response from SAMConfiguration");
        return ESP_FAIL;
    }
    
    // Read response
    uint8_t response[9];
    if (pn532_read_data(response, 9) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Validate response: [STATUS] 00 00 FF 02 FE D5 15 16 00
    if (response[6] == PN532_PN532TOHOST && 
        response[7] == (PN532_COMMAND_SAMCONFIGURATION + 1)) {
        ESP_LOGI(TAG, "✓ SAMConfiguration successful!");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Invalid SAM response");
    return ESP_FAIL;
}

// Read passive target (NFC card detection)
esp_err_t pn532_read_passive_target(uint8_t *uid, uint8_t *uid_len) {
    ESP_LOGI(TAG, "\n--- Detecting NFC Card ---");
    
    uint8_t cmd[] = {
        PN532_COMMAND_INLISTPASSIVETARGET,
        0x01,  // Max 1 card
        PN532_MIFARE_ISO14443A  // 106 kbps type A (ISO/IEC14443 Type A)
    };
    
    if (pn532_write_cmd_check_ack(cmd, sizeof(cmd)) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Wait for response (card detection can take longer)
    vTaskDelay(pdMS_TO_TICKS(100));
    if (pn532_wait_ready(1000) != ESP_OK) {
        ESP_LOGW(TAG, "No card detected (timeout)");
        return ESP_ERR_TIMEOUT;
    }
    
    // Read response
    uint8_t response[64];
    if (pn532_read_data(response, sizeof(response)) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Parse response frame structure:
    // [0]      = Ready status (0x01)
    // [1-5]    = Frame header (00 00 FF LEN LCS)
    // [6]      = TFI (D5)
    // [7]      = Command+1 (4B)
    // [8]      = NbTg (number of tags)
    // [9]      = Tg (target number)
    // [10-11]  = SENS_RES
    // [12]     = SEL_RES
    // [13]     = NFCID Length
    // [14+]    = NFCID (UID)
    
    if (response[6] == PN532_PN532TOHOST && 
        response[7] == (PN532_COMMAND_INLISTPASSIVETARGET + 1)) {
        
        uint8_t num_tags = response[8];
        
        if (num_tags == 0) {
            ESP_LOGW(TAG, "No cards found");
            return ESP_ERR_NOT_FOUND;
        }
        
        // Extract UID length and UID
        *uid_len = response[13];  // Correct position for UID length
        
        if (*uid_len > 10) {
            ESP_LOGE(TAG, "UID too long: %d (invalid response)", *uid_len);
            ESP_LOGI(TAG, "Response dump:");
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, response, 20, ESP_LOG_INFO);
            return ESP_FAIL;
        }
        
        // Copy UID (starts at response[14])
        memcpy(uid, &response[14], *uid_len);
        
        ESP_LOGI(TAG, "✓ Card detected!");
        ESP_LOGI(TAG, "  Num Tags:   %d", num_tags);
        ESP_LOGI(TAG, "  Target:     %d", response[9]);
        ESP_LOGI(TAG, "  SENS_RES:   %02X %02X", response[10], response[11]);
        ESP_LOGI(TAG, "  SEL_RES:    %02X", response[12]);
        ESP_LOGI(TAG, "  UID Length: %d bytes", *uid_len);
        ESP_LOGI(TAG, "  UID:");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, uid, *uid_len, ESP_LOG_INFO);
        
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

// Get Firmware Version
esp_err_t pn532_get_firmware_version(void) {
    ESP_LOGI(TAG, "\n--- Get Firmware Version ---");
    
    uint8_t cmd[] = {PN532_COMMAND_GETFIRMWAREVERSION};
    
    if (pn532_write_cmd_check_ack(cmd, sizeof(cmd)) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Wait for response ready
    vTaskDelay(pdMS_TO_TICKS(100));
    if (pn532_wait_ready(500) != ESP_OK) {
        ESP_LOGE(TAG, "No response from GetFirmwareVersion");
        return ESP_FAIL;
    }
    
    // Read response
    uint8_t response[13];
    if (pn532_read_data(response, 13) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Validate and parse: [STATUS] 00 00 FF 06 FA D5 03 IC VER REV SUPPORT
    if (response[1] == 0x00 && response[2] == 0x00 && response[3] == 0xFF &&
        response[6] == PN532_PN532TOHOST && 
        response[7] == (PN532_COMMAND_GETFIRMWAREVERSION + 1)) {
        
        uint8_t ic = response[8];
        uint8_t ver = response[9];
        uint8_t rev = response[10];
        uint8_t support = response[11];
        
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════╗");
        ESP_LOGI(TAG, "║   PN532 Firmware Information   ║");
        ESP_LOGI(TAG, "╠════════════════════════════════╣");
        ESP_LOGI(TAG, "║ Chip:     PN5%02X                ║", ic);
        ESP_LOGI(TAG, "║ Version:  %d.%-2d                 ║", ver, rev);
        ESP_LOGI(TAG, "║ Support:  0x%02X                ║", support);
        ESP_LOGI(TAG, "╚════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Invalid firmware response");
    return ESP_FAIL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   PN532 NFC Reader Test Program     ║");
    ESP_LOGI(TAG, "║   ESP-IDF v%s                  ║", esp_get_idf_version());
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    // Initialize GPIO
    pn532_reset_gpio_init();
    
    // Initialize I2C
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_LOGI(TAG, "  Port: I2C_NUM_0");
    ESP_LOGI(TAG, "  SDA:  GPIO%d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "  SCL:  GPIO%d", I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "  Freq: %d Hz", I2C_MASTER_FREQ_HZ);
    ESP_LOGI(TAG, "  Addr: 0x%02X", PN532_I2C_ADDRESS);
    
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "✓ I2C initialized\n");
    
    // Hardware reset
    pn532_reset_gpio();
    
    // Wakeup sequence
    pn532_wakeup();
    
    // Test communication
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "Testing PN532 Communication");
    ESP_LOGI(TAG, "========================================\n");
    
    // Get firmware version
    ret = pn532_get_firmware_version();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ GetFirmwareVersion FAILED");
        ESP_LOGE(TAG, "\nTroubleshooting:");
        ESP_LOGE(TAG, "  1. Check DIP switches: [OFF][ON] for I2C mode");
        ESP_LOGE(TAG, "  2. Verify wiring connections");
        ESP_LOGE(TAG, "  3. Check 3.3V power supply");
        return;
    }
    
    // Configure SAM
    ret = pn532_SAMConfiguration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ SAMConfiguration FAILED");
        return;
    }
    
    ESP_LOGI(TAG, "\n╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ✓✓✓ ALL TESTS PASSED! ✓✓✓        ║");
    ESP_LOGI(TAG, "║   PN532 is ready for NFC operations  ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝\n");
    
    // Main loop - scan for NFC cards
    ESP_LOGI(TAG, "Starting NFC card detection loop...");
    ESP_LOGI(TAG, "Place an NFC card near the reader\n");
    
    uint8_t uid[10];
    uint8_t uid_len = 0;
    
    while(1) {
        // Try to detect a card
        esp_err_t result = pn532_read_passive_target(uid, &uid_len);
        
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "\n╔══════════════════════════════════════╗");
            ESP_LOGI(TAG, "║          CARD DETECTED!              ║");
            ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
            
            // Print UID as hex string
            printf("Card UID: ");
            for (int i = 0; i < uid_len; i++) {
                printf("%02X", uid[i]);
                if (i < uid_len - 1) printf(":");
            }
            printf("\n\n");
            
            // Wait for card to be removed
            ESP_LOGI(TAG, "Remove card to scan again...\n");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        
        // Scan every 500ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}