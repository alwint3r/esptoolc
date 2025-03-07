#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef int serial_port_t;

typedef enum {
  ESP_SUCCESS = 0,
  ESP_ERR_INVALID_PORT = -1,
  ESP_ERR_PORT_OPEN = -2,
  ESP_ERR_PORT_CONFIG = -3,
  ESP_ERR_RESET_FAILED = -4,
  ESP_ERR_READ_FAILED = -5,
  ESP_ERR_WRITE_FAILED = -6,
  ESP_ERR_TIMEOUT = -7,
  ESP_ERR_INVALID_RESPONSE = -8,
  ESP_ERR_INVALID_ARG = -9,
} esp_error_t;

typedef enum {
  RESET_TYPE_CLASSIC = 0,
  RESET_TYPE_UNIX,
  RESET_TYPE_HARD,
} reset_type_t;

typedef struct {
  const char* port_name;
  int baud_rate;
  bool esp32r0_delay;
  reset_type_t reset_type;
} esp_port_config_t;

esp_error_t esp_port_open(serial_port_t* port, esp_port_config_t* config);
esp_error_t esp_port_close(serial_port_t port);
esp_error_t esp_enter_download_mode(serial_port_t port,
                                    esp_port_config_t* config);
esp_error_t esp_reset(serial_port_t port, esp_port_config_t* config);

esp_error_t esp_read_timeout(serial_port_t port, uint8_t* buffer, size_t size,
                             int timeout_ms, size_t* out_size);
esp_error_t esp_write(serial_port_t port, const uint8_t* data, size_t size);
esp_error_t esp_discard_input(serial_port_t port);
