#ifndef PN532_H
#define PN532_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize PN532 module via SPI
 * 
 * @param miso_pin MISO GPIO pin
 * @param mosi_pin MOSI GPIO pin
 * @param clk_pin SCK GPIO pin
 * @param cs_pin Chip Select GPIO pin
 * @return true if initialization successful
 */
bool pn532_init(int miso_pin, int mosi_pin, int clk_pin, int cs_pin);

/**
 * @brief Configure PN532 SAM (Security Access Module)
 * 
 * @return true if configuration successful
 */
bool pn532_sam_config(void);

/**
 * @brief Read UID from a passive NFC target (card/tag)
 * 
 * @param uid Buffer to store the UID
 * @param uid_len Pointer to store the UID length
 * @return true if card detected and UID read successfully
 */
bool pn532_read_passive_target(uint8_t *uid, uint8_t *uid_len);

#endif // PN532_H
