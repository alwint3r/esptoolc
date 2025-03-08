#include "esp_chip.h"
#include "os_hal.h"

#include <stdio.h>

#define DEFAULT_RESET_DELAY 100

esp_error_t esp_enter_download_mode(serial_port_t port) {
  if (!port) {
    return ESP_ERR_INVALID_PORT;
  }

  reset_type_t reset_types[] = {
      RESET_TYPE_USB_JTAG,
      RESET_TYPE_USB_OTG,
      RESET_TYPE_UNIX,
  };

  char response_buf[1024];
  size_t reset_types_count = sizeof(reset_types) / sizeof(reset_type_t);
  for (size_t i = 0; i < reset_types_count; i++) {
    esp_error_t err = esp_reset(port, reset_types[i]);
    if (err != ESP_SUCCESS) {
      fprintf(stderr, "Failed to perform reset sequence: %d\n", err);
      continue;
    }

    os_sleep_ms(500);

    size_t length;
    while ((err = esp_read_timeout(port, response_buf, sizeof(response_buf), 100, &length)) == ESP_SUCCESS) {
      if (length < 1) {
        fprintf(stderr, "Failed to read response: %d\n", length);
        continue;
      }
      if (strstr(response_buf, "waiting for download") != NULL) {
        return ESP_SUCCESS;
      }
    }

    os_sleep_ms(500);
  }

  return ESP_ERR_RESET_FAILED;
}

esp_error_t esp_hard_reset(serial_port_t port) {
  esp_error_t err;
  // ensure IO0 is HIGH to prevent it entering the download mode
  if ((err = esp_set_dtr(port, false)) != ESP_SUCCESS) {
    return err;
  }

  if ((err = esp_set_rts(port, true)) != ESP_SUCCESS) {
    return err;
  }

  os_sleep_ms(100);

  if ((err = esp_set_rts(port, false)) != ESP_SUCCESS) {
    return err;
  }

  return ESP_SUCCESS;
}

esp_error_t esp_reset(serial_port_t port, reset_type_t reset_type) {
  esp_error_t err;
  int reset_delay = DEFAULT_RESET_DELAY;

  if (reset_type == RESET_TYPE_UNIX) {
    // Unix-style reset sequence with both pins toggled simultaneously
    if ((err = esp_set_dtr_rts(port, false, false)) != ESP_SUCCESS) return err;
    if ((err = esp_set_dtr_rts(port, true, true)) != ESP_SUCCESS) return err;
    if ((err = esp_set_dtr_rts(port, false, true)) != ESP_SUCCESS)
      return err;  // IO0=HIGH & EN=LOW
    os_sleep_ms(reset_delay);
    if ((err = esp_set_dtr_rts(port, true, false)) != ESP_SUCCESS)
      return err;  // IO0=LOW & EN=HIGH
    os_sleep_ms(reset_delay);
    if ((err = esp_set_dtr_rts(port, false, false)) != ESP_SUCCESS)
      return err;  // IO0=HIGH
    if ((err = esp_set_dtr(port, false)) != ESP_SUCCESS)
      return err;  // Ensure IO0=HIGH
  } else if (reset_type == RESET_TYPE_CLASSIC) {
    // Classic reset sequence
    if ((err = esp_set_dtr(port, false)) != ESP_SUCCESS) return err;  // IO0=HIGH
    if ((err = esp_set_rts(port, true)) != ESP_SUCCESS)
      return err;  // EN=LOW, reset
    os_sleep_ms(reset_delay);
    if ((err = esp_set_dtr(port, true)) != ESP_SUCCESS) return err;  // IO0=LOW
    if ((err = esp_set_rts(port, false)) != ESP_SUCCESS)
      return err;  // EN=HIGH, out of reset
    os_sleep_ms(reset_delay);
    if ((err = esp_set_dtr(port, false)) != ESP_SUCCESS)
      return err;  // IO0=HIGH, done
  } else if (reset_type == RESET_TYPE_USB_JTAG) {
    if ((err = esp_set_dtr_rts(port, false, false)) != ESP_SUCCESS) return err;
    if ((err = esp_set_dtr(port, true)) != ESP_SUCCESS) return err;  // IO0=LOW
    os_sleep_ms(reset_delay);
    if ((err = esp_set_rts(port, true)) != ESP_SUCCESS) return err;  // EN=LOW
    if ((err = esp_set_dtr(port, false)) != ESP_SUCCESS) return err;
    if ((err = esp_set_rts(port, false)) != ESP_SUCCESS) return err;
  } else if (reset_type == RESET_TYPE_USB_OTG) {
    if ((err = esp_set_dtr_rts(port, false, false)) != ESP_SUCCESS) return err;
    os_sleep_ms(reset_delay);
    if ((err = esp_set_dtr_rts(port, true, true)) != ESP_SUCCESS) return err;
    os_sleep_ms(reset_delay);
    if ((err = esp_set_dtr_rts(port, false, false)) != ESP_SUCCESS) return err;
  } else {
    esp_hard_reset(port);
  }

  return ESP_SUCCESS;
}