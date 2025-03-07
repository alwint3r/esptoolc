#include "esp_command_sync.h"

#include <string.h>

#include "slip_writer.h"

typedef struct __attribute__((packed)) {
  uint8_t dir;
  uint8_t cmd;
  uint16_t size;
  uint32_t checksum;
  uint8_t data[36];
} esp_command_sync_t;

int esp_command_sync_encoded(uint8_t* output) {
  uint8_t data[36] = {0x07, 0x07, 0x12, 0x20};
  memset(data + 4, 0x55, 32);

  uint8_t cmd_buf[44];
  esp_command_sync_t* cmd = (esp_command_sync_t*)cmd_buf;
  cmd->dir = 0x00;
  cmd->cmd = ESP_CMD_SYNC;
  cmd->size = 36;
  cmd->checksum = 0;
  memcpy(cmd->data, data, 36);

  return slip_write(output, cmd_buf, sizeof(cmd_buf));
}