#include "esp_command.h"

#include <stdbool.h>

uint8_t esp_command_checksum(uint8_t *data, size_t length) {
  uint8_t checksum = 0xEF;
  for (size_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}