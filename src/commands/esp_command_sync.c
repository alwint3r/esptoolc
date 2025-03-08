#include "esp_command_sync.h"

#include <string.h>

#include "esp_command.h"
#include "slip_writer.h"

int esp_command_sync_encoded(uint8_t* output) {
  uint8_t data[36] = {0x07, 0x07, 0x12, 0x20};
  memset(data + 4, 0x55, 32);

  uint8_t cmd_buf[44];
  esp_cmd_header_t* header = (esp_cmd_header_t*)cmd_buf;
  header->direction = 0x00;
  header->command = ESP_CMD_SYNC;
  header->data_length = 36;
  header->checksum = 0;
  memcpy(cmd_buf + sizeof(esp_cmd_header_t), data, 36);

  return slip_write(output, cmd_buf, sizeof(cmd_buf));
}