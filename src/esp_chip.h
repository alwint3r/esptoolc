#pragma once

#include "esp_err.h"
#include "esp_serial_port.h"

#define ESP_REG_MAGIC_VALUE_ADDR 0x40001000

#define ESP8266_MAGIC_VALUE 0xFFF0C101
#define ESP32_MAGIC_VALUE 0x00F01D83
#define ESP32S2_MAGIC_VALUE 0x000007C6

#define ESP32C3_CHIP_ID 0x00000005
#define ESP32S3_CHIP_ID 0x00000009

#define ESP_UART_CLKDIV_MASK 0xFFFFF

#define ESP_ROM_DEFAULT_BAUD 115200

typedef enum {
  RESET_TYPE_CLASSIC = 0,
  RESET_TYPE_UNIX,
  RESET_TYPE_HARD,
  RESET_TYPE_USB_JTAG,
  RESET_TYPE_USB_OTG,
} reset_type_t;

typedef enum {
  ESP_CHIP_ESP8266,
  ESP_CHIP_ESP32,
  ESP_CHIP_ESP32S2,
  ESP_CHIP_ESP32S3,
  ESP_CHIP_ESP32C3,
  ESP_CHIP_UNKNOWN,
} esp_chip_type_t;

typedef struct __attribute__((packed)) {
  uint32_t flags;
  uint8_t flash_crypt_cnt;
  uint8_t flash_crypt_key[7];
} esp_chip_sec_info_t;

typedef struct __attribute__((packed)) {
  esp_chip_sec_info_t sec_info;
  bool is_chip_id_valid;
  uint32_t chip_id;
  uint32_t api_version;
} esp_chip_sec_resp_t;

/**
 * @brief Reset an ESP device connected to a serial port.
 *
 * This function triggers a reset on an ESP device based on the specified reset
 * type. The reset can be a hard reset (using the hardware reset line) or a soft
 * reset (using a software command sequence).
 *
 * @param port The serial port connected to the ESP device
 * @param reset_type The type of reset to perform depending on how the device is
 * connected
 *
 * @return esp_error_t
 *     - ESP_OK on success and the device is reset
 */
esp_error_t esp_chip_read_reg(serial_port_t port, uint32_t addr,
                              uint32_t *value, uint8_t *data_out,
                              size_t *data_out_len);

/**
 * @brief Reads the magic value from an ESP chip.
 *
 * This function communicates with an ESP chip over the specified serial port
 * to read its magic value, which can be used to identify the chip type for
 * older chip type like ESP8266, ESP32, and ESP32-S2.
 *
 * @param port The serial port connected to the ESP chip
 * @param value Pointer to store the read magic value
 *
 * @return ESP_OK if successful, or an error code if the operation failed
 */
esp_error_t esp_chip_read_magic_value(serial_port_t port, uint32_t *value);

/**
 * @brief Get security information from an ESP chip.
 *
 * This function retrieves security-related information from an ESP chip
 * connected to the specified serial port.
 * This information is used to identify the chip type for newer chip types like
 * ESP32-S3 and ESP32-C3.
 *
 * @param port The serial port where the ESP chip is connected.
 * @param resp Pointer to a structure where the security response will be
 * stored.
 *
 * @return esp_error_t
 *         - ESP_OK: Security information retrieved successfully
 */
esp_error_t esp_chip_get_security_info(serial_port_t port,
                                       esp_chip_sec_resp_t *resp);

esp_error_t esp_chip_get_type_from_magic(serial_port_t port,
                                         esp_chip_type_t *chip_type);

const char *esp_chip_type_str(esp_chip_type_t chip_type);

esp_error_t esp_chip_get_type_from_sec_info(serial_port_t port,
                                            esp_chip_type_t *chip_type);

/**
 * @brief Get the ESP chip type from the device connected to the given serial
 * port.
 *
 * @warning It is SLOW for ESP32 chip because the tool will try to read
 * the security info first, reset the bootloader communication, and then read
 * the magic value.
 *
 * @param port The serial port connected to the ESP device
 * @param chip_type Pointer to store the detected chip type
 * @return esp_error_t ESP_OK on success, or an error code on failure
 */
esp_error_t esp_chip_get_type(serial_port_t port, esp_chip_type_t *chip_type);
