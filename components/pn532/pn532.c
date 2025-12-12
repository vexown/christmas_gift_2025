#include "pn532.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "PN532";
static spi_device_handle_t spi;

// PN532 Commands
#define PN532_COMMAND_SAMCONFIGURATION   (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)

#define PN532_PREAMBLE  (0x00)
#define PN532_STARTCODE1 (0x00)
#define PN532_STARTCODE2 (0xFF)
#define PN532_POSTAMBLE (0x00)

// PN532 SPI interface
#define PN532_SPI_STATREAD  (0x02)
#define PN532_SPI_DATAWRITE (0x01)
#define PN532_SPI_DATAREAD  (0x03)

static void pn532_write_command(const uint8_t *cmd, uint8_t cmd_len) {
    uint8_t buffer[64];
    uint8_t idx = 0;
    
    // Bounds check
    if (cmd_len > 50) {
        ESP_LOGE(TAG, "Command too long");
        return;
    }
    
    // SPI "data write" prefix
    buffer[idx++] = PN532_SPI_DATAWRITE;

    buffer[idx++] = PN532_PREAMBLE;
    buffer[idx++] = PN532_STARTCODE1;
    buffer[idx++] = PN532_STARTCODE2;
    buffer[idx++] = (uint8_t)(cmd_len + 1);
    buffer[idx++] = (uint8_t)(~(cmd_len + 1) + 1);
    buffer[idx++] = 0xD4;
    
    uint8_t checksum = 0xD4;
    for (uint8_t i = 0; i < cmd_len; i++) {
        buffer[idx++] = cmd[i];
        checksum = (uint8_t)(checksum + cmd[i]);
    }
    
    buffer[idx++] = (uint8_t)(~checksum + 1);
    buffer[idx++] = PN532_POSTAMBLE;
    
    // SPI driver handles CS automatically
    spi_transaction_t t = {
        .length = idx * 8,
        .tx_buffer = buffer,
    };
    spi_device_transmit(spi, &t);
}

static bool pn532_is_data_ready(void) {
    uint8_t tx[2] = {PN532_SPI_STATREAD, 0x00};
    uint8_t rx[2] = {0};
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(spi, &t);
    
    return (rx[1] & 0x01) != 0;
}

static bool pn532_read_response(uint8_t *response, uint8_t *response_len, uint8_t max_len) {
    // Read loop: after a command, PN532 returns ACK first, then the actual response.
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t tx_buffer[64];
        uint8_t rx_buffer[64] = {0};

        if (max_len > sizeof(rx_buffer)) {
            max_len = sizeof(rx_buffer);
        }

        memset(tx_buffer, 0x00, sizeof(tx_buffer));
        tx_buffer[0] = PN532_SPI_DATAREAD;

        // Poll for data ready
        uint32_t timeout = 100; // 100ms timeout
        while (!pn532_is_data_ready() && timeout > 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            timeout--;
        }
        if (timeout == 0) {
            ESP_LOGE(TAG, "Timeout waiting for PN532 response");
            return false;
        }

        spi_transaction_t t = {
            .length = max_len * 8,
            .rx_buffer = rx_buffer,
            .tx_buffer = tx_buffer,
        };
        spi_device_transmit(spi, &t);

        // Find start of frame: 00 00 FF (start from index 1, skip garbage byte)
        uint8_t idx = 1;
        while (idx + 2 < max_len && !(rx_buffer[idx] == 0x00 && rx_buffer[idx + 1] == 0x00 && rx_buffer[idx + 2] == 0xFF)) {
            idx++;
        }
        if (idx + 5 >= max_len) {
            return false;
        }

        uint8_t len = rx_buffer[idx + 3];
        uint8_t lcs = rx_buffer[idx + 4];

        // ACK frame: 00 00 FF 00 FF 00
        if (len == 0x00 && lcs == 0xFF) {
            continue;
        }

        if ((uint8_t)(len + lcs) != 0x00) {
            ESP_LOGE(TAG, "Invalid length checksum");
            return false;
        }

        // Need: TFI+DATA (len bytes) + DCS (1) + POSTAMBLE (1)
        if ((uint32_t)idx + 5u + (uint32_t)len + 2u > (uint32_t)max_len) {
            ESP_LOGE(TAG, "Response too large/short buffer");
            return false;
        }

        // Validate DCS: sum(TFI+DATA) + DCS == 0
        uint8_t sum = 0;
        for (uint8_t i = 0; i < len; i++) {
            sum = (uint8_t)(sum + rx_buffer[idx + 5 + i]);
        }
        uint8_t dcs = rx_buffer[idx + 5 + len];
        if ((uint8_t)(sum + dcs) != 0x00) {
            ESP_LOGE(TAG, "Invalid data checksum");
            return false;
        }

        // TFI should be 0xD5 (PN532 -> host)
        if (rx_buffer[idx + 5] != 0xD5) {
            ESP_LOGE(TAG, "Unexpected TFI: 0x%02X", rx_buffer[idx + 5]);
            return false;
        }

        // Copy payload excluding TFI
        if (len < 1) {
            ESP_LOGE(TAG, "Invalid response length");
            return false;
        }

        uint8_t payload_len = (uint8_t)(len - 1);
        if (payload_len > max_len) {
            ESP_LOGE(TAG, "Payload too large");
            return false;
        }

        memcpy(response, &rx_buffer[idx + 6], payload_len);
        *response_len = payload_len;
        return true;
    }

    return false;
}

bool pn532_init(int miso_pin, int mosi_pin, int clk_pin, int cs_pin_num) {
    esp_err_t ret;
    
    spi_bus_config_t buscfg = {
        .miso_io_num = miso_pin,
        .mosi_io_num = mosi_pin,
        .sclk_io_num = clk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = cs_pin_num,
        .queue_size = 1,
        .flags = 0, // MSB-first
    };
    
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return false;
    }
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return false;
    }
    
    ESP_LOGI(TAG, "SPI initialized");
    return true;
}

bool pn532_sam_config(void) {
    uint8_t cmd[] = {PN532_COMMAND_SAMCONFIGURATION, 0x01, 0x14, 0x01};
    pn532_write_command(cmd, sizeof(cmd));
    
    uint8_t response[20];
    uint8_t resp_len;
    
    if (pn532_read_response(response, &resp_len, sizeof(response))) {
        if (resp_len >= 1 && response[0] == 0x15) {
            ESP_LOGI(TAG, "SAM configured");
            return true;
        }
        ESP_LOGE(TAG, "Unexpected SAM response");
        return false;
    }
    
    ESP_LOGE(TAG, "Failed to configure SAM");
    return false;
}

bool pn532_read_passive_target(uint8_t *uid, uint8_t *uid_len) {
    uint8_t cmd[] = {PN532_COMMAND_INLISTPASSIVETARGET, 0x01, 0x00};
    pn532_write_command(cmd, sizeof(cmd));
    
    uint8_t response[40];
    uint8_t resp_len;
    
    if (pn532_read_response(response, &resp_len, sizeof(response))) {
        // Response format: [CMD+1, NbTg, Tg, Sens(1), UID_len, UID[...]]
        if (resp_len < 7 || response[0] != 0x4B) {
            ESP_LOGD(TAG, "Invalid response format");
            return false;
        }
        
        uint8_t num_targets = response[1];
        if (num_targets == 0) {
            return false;  // No targets found
        }
        
        uint8_t uid_len_field = response[6];
        if (uid_len_field > 10 || uid_len_field == 0) {
            ESP_LOGE(TAG, "Invalid UID length: %d", uid_len_field);
            return false;
        }
        
        // Bounds check
        if (7 + uid_len_field > resp_len) {
            ESP_LOGE(TAG, "Response too short for UID");
            return false;
        }
        
        *uid_len = uid_len_field;
        memcpy(uid, &response[7], *uid_len);
        return true;
    }
    return false;
}
