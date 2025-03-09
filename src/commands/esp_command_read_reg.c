#include "esp_command_read_reg.h"

#include <string.h>

#include "esp_command.h"
#include "slip_writer.h"

#define ESP_READ_REG_DATA_SIZE 4

int32_t esp_command_read_reg_encoded(uint8_t *output, uint32_t addr) {
  uint8_t command_buf[ESP_CMD_HEAD_SIZE + ESP_READ_REG_DATA_SIZE];
  esp_cmd_header_t *cmd_header = (esp_cmd_header_t *)command_buf;
  cmd_header->direction = ESP_CMD_DIR_CMD;
  cmd_header->command = ESP_CMD_READ_REG;
  cmd_header->data_length = ESP_READ_REG_DATA_SIZE;
  cmd_header->checksum = 0;

  uint32_t *reg = (uint32_t *)&command_buf[8];
  *reg = addr;

  return slip_write(output, command_buf, sizeof(command_buf));
}
