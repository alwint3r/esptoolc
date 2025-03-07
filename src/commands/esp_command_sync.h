#pragma once

#include <inttypes.h>
#include <string.h>

#define ESP_CMD_SYNC 0x08

typedef struct __attribute__((packed)) {
  uint8_t dir;
  uint8_t cmd;
  uint16_t size;
  uint32_t value;
} esp_command_sync_response_t;

int esp_command_sync_encoded(uint8_t* output);
