#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../esp_err.h"

#define ESP32_EFUSE_RD_REG_BASE 0x3FF5A000
#define ESP32_EFUSE_RD_OFF_MAC_1 2
#define ESP32_EFUSE_RD_OFF_MAC_2 1
#define ESP32_EFUSE_RD_OFF_PKG_VER 3
#define ESP32_EFUSE_RD_OFF_MINOR_REV 5
#define ESP32_DR_REG_SYSCON_BASE 0x3FF66000
#define ESP32_APB_CTL_DATE_ADDR (ESP32_DR_REG_SYSCON_BASE + 0x7C)

#define ESP32_APB_CTL_DATE_V 1
#define ESP32_APB_CTL_DATE_S 31

typedef enum {
  ESP32_NAME_ESP32_S0WDQ6,
  ESP32_NAME_ESP32_D0WDQ6_V3,
  ESP32_NAME_ESP32_D0WDQ6,
  ESP32_NAME_ESP32_S0WD,
  ESP32_NAME_ESP32_D0WD_V3,
  ESP32_NAME_ESP32_D0WD,
  ESP32_NAME_ESP32_D2WD,
  ESP32_NAME_ESP32_U4WDH,
  ESP32_NAME_ESP32_PICO_V3,
  ESP32_NAME_ESP32_PICO_D4,
  ESP32_NAME_ESP32_PICO_V3_02,
  ESP32_NAME_ESP32_D0WDR2_V3,
  ESP32_NAME_ESP32_UNKNOWN,
} esp32_chip_name_t;

typedef struct {
  esp32_chip_name_t name;
  uint8_t major_rev;
  uint8_t minor_rev;
} esp32_chip_desc_t;

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

esp_error_t esp32_get_pkg_version(int port, uint32_t *out);

esp_error_t esp32_get_minor_revision(int port, uint32_t *out);

esp_error_t esp32_get_major_revision(int port, uint32_t *out);

esp_error_t esp32_is_single_core(int port, bool *out);

/**
 * @brief Get the chip description for an ESP32 device.
 *
 * This function retrieves the chip description for an ESP32 device connected
 * to the specified port.
 *
 * @param port The port number of the connected ESP32 device.
 * @param out Pointer to the esp32_chip_desc_t structure where the chip
 *            description will be stored.
 *
 * @return esp_error_t
 *         - ESP_SUCCESS if successful
 *         - Other error codes on failure
 */
esp_error_t esp32_get_chip_desc(int port, esp32_chip_desc_t *out);

const char *esp32_chip_name_str(esp32_chip_name_t name);
