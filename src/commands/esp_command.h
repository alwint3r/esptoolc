#pragma once

#include <stdint.h>
#include <stdlib.h>

typedef struct __attribute__((packed)) {
  uint8_t direction;  // 0x00 for host to chip, 0x01 for chip to host
  uint8_t command;    // Command code
  uint16_t data_length;
  uint32_t checksum;  // Checksum of the data
} esp_cmd_header_t;

typedef struct __attribute__((packed)) {
  uint8_t direction;  // 0x00 for host to chip, 0x01 for chip to host
  uint8_t command;    // Command code
  uint16_t data_length;
  uint32_t value;  // Checksum of the data
} esp_cmd_resp_header_t;

uint8_t esp_command_checksum(uint8_t *data, size_t length);