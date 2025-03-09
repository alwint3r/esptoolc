#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"

typedef int serial_port_t;


typedef struct {
  const char* port_name;
  int64_t baud_rate;
  bool esp32r0_delay;
} esp_port_config_t;

esp_error_t esp_port_open(serial_port_t* port, esp_port_config_t* config);
esp_error_t esp_port_close(serial_port_t port);
esp_error_t esp_read_timeout(serial_port_t port, uint8_t* buffer, size_t size,
                             int timeout_ms, size_t* out_size);
esp_error_t esp_write(serial_port_t port, const uint8_t* data, size_t size);
esp_error_t esp_discard_input(serial_port_t port);

esp_error_t esp_set_dtr(serial_port_t port, bool dtr);
esp_error_t esp_set_rts(serial_port_t port, bool rts);
esp_error_t esp_set_dtr_rts(serial_port_t port, bool dtr, bool rts);
