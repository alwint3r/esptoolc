#pragma once

#include "esp_err.h"
#include "esp_serial_port.h"

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
  ESP_CHIP_UNKNOWN,
} esp_chip_type_t;

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

esp_error_t esp_chip_read_reg(serial_port_t port, uint32_t addr,
                              uint32_t *value, uint8_t *data_out,
                              size_t *data_out_len);
esp_error_t esp_chip_read_magic_value(serial_port_t port, uint32_t *value);

esp_error_t esp_chip_get_type_from_magic(serial_port_t port,
                                         esp_chip_type_t *chip_type);

const char* esp_chip_type_str(esp_chip_type_t chip_type);