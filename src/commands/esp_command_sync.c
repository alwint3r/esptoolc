#include "esp_command_sync.h"

#include <string.h>

#include "esp_command.h"
#include "slip_writer.h"

int32_t esp_command_sync_encoded(uint8_t* output) {
  uint8_t data[36] = {0x07, 0x07, 0x12, 0x20};
  memset(data + 4, 0x55, 32);

  uint8_t cmd_buf[44];
  esp_cmd_header_t* header = (esp_cmd_header_t*)cmd_buf;
  header->direction = ESP_CMD_DIR_CMD;
  header->command = ESP_CMD_SYNC_CHIP;
  header->data_length = 36;
  header->checksum = 0;
  memcpy(cmd_buf + sizeof(esp_cmd_header_t), data, 36);

  return slip_write(output, cmd_buf, sizeof(cmd_buf));
}
