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

esp_error_t esp_reset(serial_port_t port, reset_type_t reset_type);
esp_error_t esp_hard_reset(serial_port_t port);
esp_error_t esp_enter_download_mode(serial_port_t port);

