#include "esp_chip.h"

#include <stdio.h>

#include "commands/esp_command.h"
#include "commands/esp_command_read_reg.h"
#include "commands/esp_command_sync.h"
#include "osal.h"
#include "slip_reader.h"
#include "slip_writer.h"

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

  uint8_t response_buf[1024];
  size_t reset_types_count = sizeof(reset_types) / sizeof(reset_type_t);
  for (size_t i = 0; i < reset_types_count; i++) {
    esp_error_t err = esp_reset(port, reset_types[i]);
    if (err != ESP_SUCCESS) {
      fprintf(stderr, "Failed to perform reset sequence: %d\n", err);
      continue;
    }

    os_sleep_ms(500);

    size_t length;
    while ((err = esp_read_timeout(port, response_buf, sizeof(response_buf),
                                   100, &length)) == ESP_SUCCESS) {
      if (length < 1) {
        fprintf(stderr, "Failed to read response: %d\n", (int)length);
        continue;
      }
      if (strstr((char *)response_buf, "waiting for download") != NULL) {
        return ESP_SUCCESS;
      }
    }

    os_sleep_ms(500);
  }

  return ESP_ERR_RESET_FAILED;
}

esp_error_t esp_chip_hard_reset(serial_port_t port) {
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
    if ((err = esp_set_dtr(port, false)) != ESP_SUCCESS)
      return err;  // IO0=HIGH
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
    esp_chip_hard_reset(port);
  }

  return ESP_SUCCESS;
}

esp_error_t esp_chip_sync(serial_port_t port) {
  uint8_t command_out_buf[128];
  size_t command_out_len = esp_command_sync_encoded(command_out_buf);

  esp_error_t err = esp_write(port, command_out_buf, command_out_len);
  if (err != ESP_SUCCESS) {
    fprintf(stderr, "Failed to write command to ESP chip: %d\n", err);
    return err;
  }

  uint8_t response_buf[24];
  uint8_t slip_reader_buf[24];
  slip_reader_t slip_reader;

  uint8_t response_count = 0;
  uint8_t max_response_count = 8;
  slip_reader_init(&slip_reader, slip_reader_buf, sizeof(slip_reader_buf));

  bool first_ack_received = false;

  size_t maximum_retry = 100;
  for (size_t attempt = 0; attempt < maximum_retry; attempt++) {
    size_t read_length = 0;
    err = esp_read_timeout(port, response_buf, 24, 100, &read_length);
    if (err != ESP_SUCCESS) {
      if (err == ESP_ERR_TIMEOUT) {
        if (!first_ack_received) {
          err = esp_write(port, command_out_buf, command_out_len);
          if (err != ESP_SUCCESS) {
            fprintf(stderr, "Failed to write command to ESP chip\n");
            return err;
          }
        }

        continue;
      } else {
        fprintf(stderr, "Failed to read response from ESP chip: %d\n", err);
        return err;
      }
    }

    if (!first_ack_received) {
      first_ack_received = true;
    }

    for (size_t j = 0; j < read_length; j++) {
      slip_reader_state_t state =
          slip_reader_process_byte(&slip_reader, response_buf[j]);
      if (state == SLIP_READER_END) {
        esp_cmd_resp_header_t *response =
            (esp_cmd_resp_header_t *)slip_reader.buf;
        if (response->direction != ESP_CMD_DIR_RESP &&
            response->command != ESP_CMD_SYNC_CHIP) {
          fprintf(stderr, "Invalid response from ESP chip\n");
          return ESP_ERR_INVALID_RESPONSE;
        }

        if (response->value == 0x00) {
          printf("ESP chip is in flasher stub mode\n");
          return ESP_ERR_INVALID_RESPONSE;
        }

        slip_reader_reset(&slip_reader);

        response_count++;
      }
    }

    if (response_count >= max_response_count) {
      break;
    }
  }

  if (response_count == 0) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  return ESP_SUCCESS;
}

esp_error_t esp_chip_read_reg(serial_port_t port, uint32_t addr,
                              uint32_t *value, uint8_t *data_out,
                              size_t *data_out_len) {
  uint8_t slip_out[32];
  int32_t slip_out_len = esp_command_read_reg_encoded(slip_out, addr);
  if (slip_out_len < 0) {
    return ESP_ERR_INVALID_COMMAND;
  }

  uint8_t response_buf[64];
  uint8_t slip_reader_buf[64];
  slip_reader_t slip_reader;
  slip_reader_init(&slip_reader, slip_reader_buf, sizeof(slip_reader_buf));

  esp_error_t err = esp_write(port, slip_out, (size_t)slip_out_len);
  if (err != ESP_SUCCESS) {
    fprintf(stderr, "Failed to write command to ESP chip: %d\n", err);
    return err;
  }

  size_t max_attempt = 100;
  for (size_t attempt = 0; attempt < max_attempt; attempt++) {
    size_t read_length = 0;
    err = esp_read_timeout(port, response_buf, sizeof(response_buf), 100,
                           &read_length);
    if (err != ESP_SUCCESS) {
      return err;
    }
    for (size_t j = 0; j < read_length; j++) {
      slip_reader_state_t state =
          slip_reader_process_byte(&slip_reader, response_buf[j]);
      if (state == SLIP_READER_END) {
        if (slip_reader.buf[0] != ESP_CMD_DIR_RESP &&
            slip_reader.buf[1] != ESP_CMD_READ_REG) {
          fprintf(stderr, "Invalid response from ESP chip\n");
          return ESP_ERR_INVALID_RESPONSE;
        }

        uint16_t *size = (uint16_t *)&slip_reader.buf[2];
        if (value) {
          *value = *(uint32_t *)&slip_reader.buf[4];
        }

        if (data_out && data_out_len) {
          *data_out_len = *size;
          memcpy(data_out, &slip_reader.buf[8], *data_out_len);
        }
        slip_reader_reset(&slip_reader);
        return ESP_SUCCESS;
      }
    }
  }

  return ESP_ERR_TIMEOUT;
}

esp_error_t esp_chip_read_magic_value(serial_port_t port, uint32_t *value) {
  esp_error_t err =
      esp_chip_read_reg(port, ESP_REG_MAGIC_VALUE_ADDR, value, NULL, NULL);
  return err;
}

esp_error_t esp_chip_get_type_from_magic(serial_port_t port,
                                         esp_chip_type_t *chip_type) {
  uint32_t magic_value;
  esp_error_t err = esp_chip_read_magic_value(port, &magic_value);
  if (err != ESP_SUCCESS) {
    return err;
  }

  switch (magic_value) {
    case 0xFFF0C101:
      *chip_type = ESP_CHIP_ESP8266;
      break;
    case 0x00F01D83:
      *chip_type = ESP_CHIP_ESP32;
      break;
    case 0x000007C6:
      *chip_type = ESP_CHIP_ESP32S2;
      break;
    default:
      *chip_type = ESP_CHIP_UNKNOWN;
      break;
  }
  return ESP_SUCCESS;
}

const char *esp_chip_type_str(esp_chip_type_t chip_type) {
  switch (chip_type) {
    case ESP_CHIP_ESP8266:
      return "ESP8266";
    case ESP_CHIP_ESP32:
      return "ESP32";
    case ESP_CHIP_ESP32S2:
      return "ESP32-S2";
    case ESP_CHIP_ESP32C3:
      return "ESP32-C3";
    case ESP_CHIP_ESP32S3:
      return "ESP32-S3";
    default:
      return "Unknown";
  }
}

static __attribute__((unused)) void dump_hex(const uint8_t *data,
                                             size_t length) {
  for (size_t i = 0; i < length; i++) {
    printf("%02X ", data[i]);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

esp_error_t esp_chip_get_security_info(serial_port_t port,
                                       esp_chip_sec_resp_t *resp) {
  uint8_t cmd_buffer[ESP_CMD_HEAD_SIZE];
  esp_cmd_header_t *cmd_header = (esp_cmd_header_t *)cmd_buffer;
  cmd_header->direction = ESP_CMD_DIR_CMD;
  cmd_header->command = ESP_CMD_GET_SECURITY_INFO;
  cmd_header->data_length = 0;
  cmd_header->checksum = 0;

  uint8_t slip_out_buf[32];
  int32_t slip_out_len =
      slip_write(slip_out_buf, cmd_buffer, sizeof(cmd_buffer));
  if (slip_out_len < 0) {
    return ESP_ERR_INVALID_COMMAND;
  }

  esp_error_t err = esp_write(port, slip_out_buf, (size_t)slip_out_len);
  if (err != ESP_SUCCESS) {
    fprintf(stderr, "Failed to write command to ESP chip: %d\n", err);
    return err;
  }

  uint8_t response_buf[64];
  uint8_t slip_reader_buf[64];
  slip_reader_t slip_reader;

  slip_reader_init(&slip_reader, slip_reader_buf, sizeof(slip_reader_buf));
  size_t max_attempt = 100;

  for (size_t attempt = 0; attempt < max_attempt; attempt++) {
    size_t read_length = 0;
    err = esp_read_timeout(port, response_buf, sizeof(response_buf), 100,
                           &read_length);
    if (err != ESP_SUCCESS) {
      return err;
    }
    for (size_t j = 0; j < read_length; j++) {
      slip_reader_state_t state =
          slip_reader_process_byte(&slip_reader, response_buf[j]);
      if (state == SLIP_READER_END) {
        if (slip_reader.buf[0] != ESP_CMD_DIR_RESP ||
            slip_reader.buf[1] != ESP_CMD_GET_SECURITY_INFO) {
          return ESP_ERR_INVALID_RESPONSE;
        }

        esp_cmd_resp_header_t *resp_hdr =
            (esp_cmd_resp_header_t *)slip_reader.buf;
        uint8_t *data = slip_reader.buf + ESP_CMD_HEAD_SIZE;
        uint8_t status_byte = data[resp_hdr->data_length - 2];
        if (status_byte != 0) {
          uint8_t reason = data[resp_hdr->data_length - 1];
          fprintf(stderr,
                  "Failed to get security info from ESP chip. Reason=%d\n",
                  (int)reason);
          return ESP_ERR_READ_FAILED;
        }

        if (!resp) {
          return ESP_SUCCESS;
        }

        esp_chip_sec_info_t *sec_info = (esp_chip_sec_info_t *)data;
        memcpy(&resp->sec_info, sec_info, sizeof(esp_chip_sec_info_t));
        if ((resp_hdr->data_length - 2) == 12) {
          return ESP_SUCCESS;
        }

        resp->is_chip_id_valid = true;
        memcpy(&resp->chip_id, &data[12], sizeof(uint32_t));
        memcpy(&resp->api_version, &data[16], sizeof(uint32_t));

        slip_reader_reset(&slip_reader);
        return ESP_SUCCESS;
      }
    }
  }
  return ESP_ERR_TIMEOUT;
}

esp_error_t esp_chip_get_type_from_sec_info(serial_port_t port,
                                            esp_chip_type_t *chip_type) {
  esp_chip_sec_resp_t sec_resp;
  esp_error_t err = esp_chip_get_security_info(port, &sec_resp);
  if (err != ESP_SUCCESS) {
    return err;
  }

  if (!sec_resp.is_chip_id_valid) {
    return ESP_ERR_INVALID_COMMAND;
  }

  switch (sec_resp.chip_id) {
    case ESP32S3_CHIP_ID:
      *chip_type = ESP_CHIP_ESP32S3;
      break;
    case ESP32C3_CHIP_ID:
      *chip_type = ESP_CHIP_ESP32C3;
      break;
    default:
      *chip_type = ESP_CHIP_UNKNOWN;
      break;
  }

  return ESP_SUCCESS;
}

esp_error_t esp_chip_get_type(serial_port_t port, esp_chip_type_t *chip_type) {
  esp_error_t err = esp_chip_get_type_from_sec_info(port, chip_type);
  if (err != ESP_SUCCESS) {
    err = esp_enter_download_mode(port);
    if (err != ESP_SUCCESS) {
      return err;
    }

    err = esp_chip_sync(port);
    if (err != ESP_SUCCESS) {
      return err;
    }

    err = esp_chip_get_type_from_magic(port, chip_type);
    if (err != ESP_SUCCESS) {
      return err;
    }
  }
  return ESP_SUCCESS;
}
