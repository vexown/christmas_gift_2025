#ifndef PN532_H
#define PN532_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// PN532 Configuration
#define PN532_I2C_ADDRESS           0x24

// PN532 Commands
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)

// Card types
#define PN532_MIFARE_ISO14443A              (0x00)

// Maximum UID length
#define PN532_MAX_UID_LENGTH                10

/**
 * @brief Initialize PN532 module
 * 
 * @param i2c_port I2C port number
 * @param sda_pin SDA GPIO pin
 * @param scl_pin SCL GPIO pin
 * @param rst_pin Reset GPIO pin
 * @param freq_hz I2C frequency in Hz
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pn532_init(int i2c_port, int sda_pin, int scl_pin, int rst_pin, uint32_t freq_hz);

/**
 * @brief Deinitialize PN532 module
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pn532_deinit(void);

/**
 * @brief Perform hardware reset of PN532
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pn532_reset(void);

/**
 * @brief Wake up PN532 from power-down mode
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pn532_wakeup(void);

/**
 * @brief Get PN532 firmware version
 * 
 * @param ic Pointer to store IC version
 * @param ver Pointer to store version number
 * @param rev Pointer to store revision number
 * @param support Pointer to store support flags
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pn532_get_firmware_version(uint8_t *ic, uint8_t *ver, uint8_t *rev, uint8_t *support);

/**
 * @brief Configure PN532 SAM (Security Access Module)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pn532_sam_configuration(void);

/**
 * @brief Detect and read passive NFC target (card)
 * 
 * @param uid Buffer to store card UID (must be at least PN532_MAX_UID_LENGTH bytes)
 * @param uid_len Pointer to store actual UID length
 * @return esp_err_t ESP_OK if card detected, ESP_ERR_NOT_FOUND if no card, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t pn532_read_passive_target(uint8_t *uid, uint8_t *uid_len);

/**
 * @brief Enable verbose debug logging
 * 
 * @param enable true to enable, false to disable
 */
void pn532_set_debug(bool enable);

#endif /* PN532_H */