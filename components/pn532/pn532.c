#include "pn532.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "PN532";

// PN532 Protocol Defines
#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)
#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)

#define PN532_I2C_READY                     (0x01)
#define PN532_TIMEOUT_MS                    1000

// ACK frame
static const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

// Module state
typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    int rst_pin;
    bool debug_enabled;
    bool initialized;
} pn532_state_t;

static pn532_state_t pn532_state = {
    .bus_handle = NULL,
    .dev_handle = NULL,
    .rst_pin = -1,
    .debug_enabled = false,
    .initialized = false
};

// Private function declarations
static esp_err_t pn532_wait_ready(uint32_t timeout_ms);
static esp_err_t pn532_write_command(const uint8_t *cmd, uint8_t cmd_len);
static esp_err_t pn532_read_data(uint8_t *buffer, uint8_t len);
static esp_err_t pn532_read_ack(void);
static esp_err_t pn532_write_cmd_check_ack(const uint8_t *cmd, uint8_t len);

// Public API implementation

esp_err_t pn532_init(int i2c_port, int sda_pin, int scl_pin, int rst_pin, uint32_t freq_hz) {
    if (pn532_state.initialized) {
        ESP_LOGW(TAG, "PN532 already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing PN532...");
    ESP_LOGI(TAG, "  I2C Port: %d", i2c_port);
    ESP_LOGI(TAG, "  SDA Pin:  GPIO%d", sda_pin);
    ESP_LOGI(TAG, "  SCL Pin:  GPIO%d", scl_pin);
    ESP_LOGI(TAG, "  RST Pin:  GPIO%d", rst_pin);
    ESP_LOGI(TAG, "  Freq:     %lu Hz", freq_hz);
    ESP_LOGI(TAG, "  Address:  0x%02X", PN532_I2C_ADDRESS);

    // Configure reset GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << rst_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure reset GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    pn532_state.rst_pin = rst_pin;

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ret = i2c_new_master_bus(&bus_config, &pn532_state.bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add PN532 device to the bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PN532_I2C_ADDRESS,
        .scl_speed_hz = freq_hz,
    };
    
    ret = i2c_master_bus_add_device(pn532_state.bus_handle, &dev_config, &pn532_state.dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PN532 device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(pn532_state.bus_handle);
        return ret;
    }

    pn532_state.initialized = true;
    ESP_LOGI(TAG, "✓ PN532 initialized successfully");
    
    return ESP_OK;
}

esp_err_t pn532_deinit(void) {
    if (!pn532_state.initialized) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;
    
    if (pn532_state.dev_handle) {
        ret = i2c_master_bus_rm_device(pn532_state.dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove PN532 device: %s", esp_err_to_name(ret));
        }
        pn532_state.dev_handle = NULL;
    }

    if (pn532_state.bus_handle) {
        ret = i2c_del_master_bus(pn532_state.bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
        }
        pn532_state.bus_handle = NULL;
    }

    if (pn532_state.rst_pin >= 0) {
        gpio_reset_pin(pn532_state.rst_pin);
        pn532_state.rst_pin = -1;
    }

    pn532_state.initialized = false;
    ESP_LOGI(TAG, "PN532 deinitialized");
    
    return ret;
}

esp_err_t pn532_reset(void) {
    if (!pn532_state.initialized || pn532_state.rst_pin < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Performing hardware reset...");
    gpio_set_level(pn532_state.rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(pn532_state.rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(500)); // Hold in reset
    gpio_set_level(pn532_state.rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for chip to boot
    ESP_LOGI(TAG, "✓ Reset complete");
    
    return ESP_OK;
}

esp_err_t pn532_wakeup(void) {
    if (!pn532_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Waking up PN532...");
    
    // Send wakeup by attempting to read (PN532 wakes on I2C activity)
    uint8_t dummy[10];
    for (int i = 0; i < 3; i++) {
        i2c_master_receive(pn532_state.dev_handle, dummy, sizeof(dummy), PN532_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "✓ Wakeup sequence complete");
    return ESP_OK;
}

esp_err_t pn532_get_firmware_version(uint8_t *ic, uint8_t *ver, uint8_t *rev, uint8_t *support) {
    if (!pn532_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!ic || !ver || !rev || !support) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Getting firmware version...");
    
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
        
        *ic = response[8];
        *ver = response[9];
        *rev = response[10];
        *support = response[11];
        
        ESP_LOGI(TAG, "✓ Firmware: PN5%02X v%d.%d (support: 0x%02X)", *ic, *ver, *rev, *support);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Invalid firmware response");
    return ESP_FAIL;
}

esp_err_t pn532_sam_configuration(void) {
    if (!pn532_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Configuring SAM...");
    
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
        ESP_LOGI(TAG, "✓ SAM configured successfully");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Invalid SAM response");
    return ESP_FAIL;
}

esp_err_t pn532_read_passive_target(uint8_t *uid, uint8_t *uid_len) {
    if (!pn532_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!uid || !uid_len) {
        return ESP_ERR_INVALID_ARG;
    }

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
            return ESP_ERR_NOT_FOUND;
        }
        
        // Extract UID length and UID
        *uid_len = response[13];
        
        if (*uid_len > PN532_MAX_UID_LENGTH) {
            ESP_LOGE(TAG, "UID too long: %d bytes (max %d)", *uid_len, PN532_MAX_UID_LENGTH);
            if (pn532_state.debug_enabled) {
                ESP_LOG_BUFFER_HEX(TAG, response, 20);
            }
            return ESP_FAIL;
        }
        
        // Copy UID (starts at response[14])
        memcpy(uid, &response[14], *uid_len);
        
        if (pn532_state.debug_enabled) {
            ESP_LOGI(TAG, "✓ Card detected!");
            ESP_LOGI(TAG, "  Num Tags:   %d", num_tags);
            ESP_LOGI(TAG, "  Target:     %d", response[9]);
            ESP_LOGI(TAG, "  SENS_RES:   %02X %02X", response[10], response[11]);
            ESP_LOGI(TAG, "  SEL_RES:    %02X", response[12]);
            ESP_LOGI(TAG, "  UID Length: %d bytes", *uid_len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, uid, *uid_len, ESP_LOG_INFO);
        }
        
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

void pn532_set_debug(bool enable) {
    pn532_state.debug_enabled = enable;
    ESP_LOGI(TAG, "Debug logging %s", enable ? "enabled" : "disabled");
}

// Private function implementations

static esp_err_t pn532_wait_ready(uint32_t timeout_ms) {
    uint32_t start = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    if (pn532_state.debug_enabled) {
        ESP_LOGI(TAG, "Waiting for PN532 ready status...");
    }
    
    uint32_t poll_count = 0;
    
    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        uint8_t status;
        
        esp_err_t ret = i2c_master_receive(pn532_state.dev_handle, &status, 1, 100);
        
        poll_count++;
        
        if (ret == ESP_OK) {
            if (pn532_state.debug_enabled) {
                ESP_LOGD(TAG, "[Poll #%lu] Status: 0x%02X %s", 
                         poll_count, status, 
                         status == PN532_I2C_READY ? "(READY!)" : "(busy)");
            }
            
            if (status == PN532_I2C_READY) {
                if (pn532_state.debug_enabled) {
                    ESP_LOGI(TAG, "✓ Ready! (took %lu polls, %lu ms)", 
                             poll_count, 
                             (xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
                }
                return ESP_OK;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGE(TAG, "Timeout waiting for ready (polled %lu times)", poll_count);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t pn532_write_command(const uint8_t *cmd, uint8_t cmd_len) {
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
    
    if (pn532_state.debug_enabled) {
        ESP_LOGI(TAG, "Sending command 0x%02X (%d bytes):", cmd[0], idx);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, frame, idx, ESP_LOG_INFO);
    }
    
    esp_err_t ret = i2c_master_transmit(pn532_state.dev_handle, frame, idx, PN532_TIMEOUT_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t pn532_read_data(uint8_t *buffer, uint8_t len) {
    esp_err_t ret = i2c_master_receive(pn532_state.dev_handle, buffer, len, PN532_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        if (pn532_state.debug_enabled) {
            ESP_LOGI(TAG, "Read %d bytes:", len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, len, ESP_LOG_INFO);
        }
    } else {
        ESP_LOGE(TAG, "I2C receive failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t pn532_read_ack(void) {
    uint8_t ack_buf[7]; // +1 for status byte
    
    // Wait longer for PN532 to prepare ACK
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Wait for ready status
    uint32_t timeout = 100; // 100ms timeout
    uint32_t start = xTaskGetTickCount();
    bool ready = false;
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout)) {
        uint8_t status;
        esp_err_t ret = i2c_master_receive(pn532_state.dev_handle, &status, 1, 50);
        
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
        ESP_LOGE(TAG, "Invalid ACK frame");
        if (pn532_state.debug_enabled) {
            ESP_LOG_BUFFER_HEX(TAG, ack_buf, 7);
        }
        return ESP_FAIL;
    }
    
    if (pn532_state.debug_enabled) {
        ESP_LOGI(TAG, "✓ ACK received");
    }
    
    return ESP_OK;
}

static esp_err_t pn532_write_cmd_check_ack(const uint8_t *cmd, uint8_t len) {
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