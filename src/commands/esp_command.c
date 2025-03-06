#include "esp_command.h"

#include <stdbool.h>

// SLIP special characters
#define ESP_SLIP_END 0xC0      // End of packet marker
#define ESP_SLIP_ESC 0xDB      // Escape character
#define ESP_SLIP_ESC_END 0xDC  // Escaped END byte
#define ESP_SLIP_ESC_ESC 0xDD  // Escaped ESC byte

size_t esp_command_slip_encode(uint8_t *dest, const uint8_t *src,
                               size_t src_len) {
  size_t dest_len = 0;
  dest[dest_len++] = ESP_SLIP_END;
  for (size_t i = 0; i < src_len; i++) {
    if (src[i] == ESP_SLIP_END) {
      dest[dest_len++] = ESP_SLIP_ESC;
      dest[dest_len++] = ESP_SLIP_ESC_END;
    } else if (src[i] == ESP_SLIP_ESC) {
      dest[dest_len++] = ESP_SLIP_ESC;
      dest[dest_len++] = ESP_SLIP_ESC_ESC;
    } else {
      dest[dest_len++] = src[i];
    }
  }

  dest[dest_len++] = ESP_SLIP_END;
  return dest_len;
}

size_t esp_command_slip_decode(uint8_t *dest, const uint8_t *src,
                               size_t src_len) {
  size_t dest_len = 0;
  bool escape = false;
  for (size_t i = 0; i < src_len; i++) {
    if (escape) {
      if (src[i] == ESP_SLIP_ESC_END) {
        dest[dest_len++] = ESP_SLIP_END;
      } else if (src[i] == ESP_SLIP_ESC_ESC) {
        dest[dest_len++] = ESP_SLIP_ESC;
      }
      escape = false;
    } else {
      if (src[i] == ESP_SLIP_END) {
        continue;
      } else if (src[i] == ESP_SLIP_ESC) {
        escape = true;
      } else {
        dest[dest_len++] = src[i];
      }
    }
  }

  return dest_len;
}