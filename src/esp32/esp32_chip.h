#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../esp_err.h"

#define ESP32_EFUSE_RD_REG_BASE 0x3FF5A000

esp_error_t esp32_read_efuse_nth_word(int port, uint32_t *out, size_t n_word);

/**
 * @brief Read the MAC address from the ESP32 chip efuse.
 *
 * @param port the serial port connected to the ESP32 chip
 * @param out Pointer to the location where the MAC address will be stored. It
 * requires at least 6 bytes of space.
 * @return esp_error_t Error code indicating success or failure of the
 * operation.
 */
esp_error_t esp32_read_mac(int port, uint8_t *out);
