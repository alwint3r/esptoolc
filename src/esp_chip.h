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

esp_error_t esp_reset(serial_port_t port, reset_type_t reset_type);
esp_error_t esp_chip_hard_reset(serial_port_t port);
esp_error_t esp_enter_download_mode(serial_port_t port);

/**
 * @brief Synchronizes the ESP chip
 *
 * @param port The serial port to use for communication
 * @return esp_error_t Returns ESP_SUCCESS on successful synchronization,
 * otherwise an error code
 */
esp_error_t esp_chip_sync(serial_port_t port);

/**
 * @brief Reads a register from the ESP chip
 *
 * This function sends a read register command to the ESP chip and retrieves the register value.
 *
 * @param port The serial port connected to the ESP chip
 * @param addr The register address to read
 * @param value Pointer to store the register value (can be NULL if not needed)
 * @param data_out Optional buffer to store additional data returned by the command
 * @param data_out_len Pointer to variable that will be updated with the length of data_out
 *
 * @return
 *     - ESP_SUCCESS on successful read
 *     - ESP_ERR_INVALID_COMMAND if command encoding fails
 *     - ESP_ERR_WRITE_FAILED if writing the command to the port fails
 *     - ESP_ERR_TIMEOUT if no response is received within the timeout period
 *     - ESP_ERR_INVALID_RESPONSE if the response from the ESP chip is invalid
 * 
 */
esp_error_t esp_chip_read_reg(serial_port_t port, uint32_t addr,
                              uint32_t *value, uint8_t *data_out,
                              size_t *data_out_len);
esp_error_t esp_chip_read_magic_value(serial_port_t port, uint32_t *value);

esp_error_t esp_chip_get_security_info(serial_port_t port,
                                       esp_chip_sec_resp_t *resp);

esp_error_t esp_chip_get_type_from_magic(serial_port_t port,
                                         esp_chip_type_t *chip_type);

const char *esp_chip_type_str(esp_chip_type_t chip_type);

esp_error_t esp_chip_get_type_from_sec_info(serial_port_t port,
                                            esp_chip_type_t *chip_type);

esp_error_t esp_chip_get_type(serial_port_t port, esp_chip_type_t *chip_type);

