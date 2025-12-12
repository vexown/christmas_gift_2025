#include "pn532.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "PN532";
static spi_device_handle_t spi;
static int cs_pin;

// PN532 Commands
#define PN532_COMMAND_SAMCONFIGURATION   (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)

#define PN532_PREAMBLE  (0x00)
#define PN532_STARTCODE1 (0x00)
#define PN532_STARTCODE2 (0xFF)
#define PN532_POSTAMBLE (0x00)

static void pn532_write_command(const uint8_t *cmd, uint8_t cmd_len) {
    uint8_t buffer[64];
    uint8_t idx = 0;
    
    buffer[idx++] = PN532_PREAMBLE;
    buffer[idx++] = PN532_STARTCODE1;
    buffer[idx++] = PN532_STARTCODE2;
    buffer[idx++] = cmd_len + 1;
    buffer[idx++] = ~(cmd_len + 1) + 1;
    buffer[idx++] = 0xD4;
    
    uint8_t checksum = 0xD4;
    for (uint8_t i = 0; i < cmd_len; i++) {
        buffer[idx++] = cmd[i];
        checksum += cmd[i];
    }
    
    buffer[idx++] = ~checksum + 1;
    buffer[idx++] = PN532_POSTAMBLE;
    
    // Write data
    gpio_set_level(cs_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    spi_transaction_t t = {
        .length = idx * 8,
        .tx_buffer = buffer,
    };
    spi_device_transmit(spi, &t);
    
    gpio_set_level(cs_pin, 1);
}

static bool pn532_read_response(uint8_t *response, uint8_t *response_len, uint8_t max_len) {
    uint8_t buffer[64] = {0};
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    gpio_set_level(cs_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    spi_transaction_t t = {
        .length = max_len * 8,
        .rx_buffer = buffer,
        .tx_buffer = NULL,
    };
    spi_device_transmit(spi, &t);
    
    gpio_set_level(cs_pin, 1);
    
    // Find start of response
    uint8_t idx = 0;
    while (idx < max_len - 5 && buffer[idx] != PN532_PREAMBLE) idx++;
    
    if (buffer[idx] == PN532_PREAMBLE && 
        buffer[idx + 1] == PN532_STARTCODE1 && 
        buffer[idx + 2] == PN532_STARTCODE2) {
        uint8_t len = buffer[idx + 3];
        memcpy(response, &buffer[idx + 6], len - 1);
        *response_len = len - 1;
        return true;
    }
    
    return false;
}

bool pn532_init(int miso_pin, int mosi_pin, int clk_pin, int cs_pin_num) {
    esp_err_t ret;
    cs_pin = cs_pin_num;
    
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
        .spics_io_num = cs_pin,
        .queue_size = 7,
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
        ESP_LOGI(TAG, "SAM configured");
        return true;
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
        if (response[0] == 0x4B && response[1] > 0) {
            *uid_len = response[6];
            memcpy(uid, &response[7], *uid_len);
            return true;
        }
    }
    return false;
}
