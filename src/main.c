#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "commands/esp_command.h"
#include "commands/esp_command_sync.h"
#include "esp_chip.h"
#include "esp_serial_port.h"
#include "os_hal.h"
#include "slip_reader.h"
#include "slip_writer.h"

#define START_PORT_READER_THREAD 0

void dump_hex(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    printf("%02X ", data[i]);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

/**
 * @brief Synchronizes the ESP chip
 *
 * @param port The serial port to use for communication
 * @return esp_error_t Returns ESP_SUCCESS on successful synchronization,
 * otherwise an error code
 */
esp_error_t sync_chip(serial_port_t port);

/**
 * @brief Reads a register value from ESP chip
 *
 * This function sends a READ_REG command to the ESP device and waits for a
 * response. It uses SLIP encoding for communication with the chip.
 *
 * @param port The serial port connected to the ESP chip
 * @param addr The memory address of the register to read
 * @param value Pointer to store the register value (can be NULL if only
 * interested in data)
 * @param data_out Optional buffer to store additional data returned by the
 * command (can be NULL)
 * @param data_out_len Optional pointer to store the length of data returned
 * (can be NULL)
 *
 * @return esp_error_t ESP_SUCCESS on successful read, otherwise an error code:
 *         - ESP_ERR_TIMEOUT if no response received after multiple attempts
 *         - ESP_ERR_INVALID_RESPONSE if response format is incorrect
 */
esp_error_t read_reg(serial_port_t port, uint32_t addr, uint32_t *value,
                     uint8_t *data_out, size_t *data_out_len);

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s <port name> <baud>\n", argv[0]);
    return 1;
  }

  const char *port_name = argv[1];
  const char *baud_str = argv[2];

  int baud = atoi(baud_str);
  serial_port_t port;
  esp_port_config_t config = {
      .port_name = port_name,
      .baud_rate = baud,
      .esp32r0_delay = false,
  };

  esp_error_t err;
  if ((err = esp_port_open(&port, &config)) != ESP_SUCCESS) {
    fprintf(stderr, "Failed to open port: %d\n", err);
    return 1;
  }

  if ((err = esp_enter_download_mode(port)) != ESP_SUCCESS) {
    fprintf(stderr, "Failed to trigger download mode: %d\n", err);
    esp_port_close(port);
    return 1;
  }

  printf("Download mode triggered successfully.\n");

  // wait for the message from ESP chip to be received
  os_sleep_ms(100);

  // discard bootloader message
  esp_discard_input(port);

  err = sync_chip(port);
  if (err != ESP_SUCCESS) {
    fprintf(stderr, "Failed to sync with ESP chip: %d\n", err);
    esp_port_close(port);
    return 1;
  }
  printf("ESP chip synced successfully.\n");

  uint32_t chip_type = 0;
  err = read_reg(port, 0x40001000, &chip_type, NULL, NULL);
  if (err != ESP_SUCCESS) {
    fprintf(stderr, "Failed to read register: %d\n", err);
    esp_port_close(port);
    return 1;
  }

  switch (chip_type) {
    case 0xFFF0C101:
      printf("Found ESP8266 chip\n");
      break;
    case 0x00F01D83:
      printf("Found ESP32 chip\n");
      break;
    case 0x000007C6:
      printf("Found ESP32-S2 chip\n");
      break;
    default:
      printf("Unknown chip: 0x%08X\n", chip_type);
      break;
  }

  esp_port_close(port);
  return 0;
}

esp_error_t sync_chip(serial_port_t port) {
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
        esp_command_sync_response_t *response =
            (esp_command_sync_response_t *)slip_reader.buf;
        if (response->dir != 0x01 && response->cmd != ESP_CMD_SYNC) {
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

esp_error_t read_reg(serial_port_t port, uint32_t addr, uint32_t *value,
                     uint8_t *data_out, size_t *data_out_len) {
  uint8_t cmd[12];
  cmd[0] = 0x00;  // direction
  cmd[1] = 0x0a;  // READ_REG command
  uint16_t *size = (uint16_t *)&cmd[2];
  *size = 4;  // size of the register to read
  uint32_t *checksum = (uint32_t *)&cmd[4];
  *checksum = 0;  // checksum (not used in this example)
  uint32_t *reg = (uint32_t *)&cmd[8];
  *reg = addr;  // register address

  uint8_t slip_out[32];
  size_t slip_out_len = slip_write(slip_out, cmd, sizeof(cmd));

  uint8_t response_buf[64];
  uint8_t slip_reader_buf[64];
  slip_reader_t slip_reader;
  slip_reader_init(&slip_reader, slip_reader_buf, sizeof(slip_reader_buf));

  esp_error_t err = esp_write(port, slip_out, slip_out_len);
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
        if (slip_reader.buf[0] != 0x01 && slip_reader.buf[1] != 0x0a) {
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

  return ESP_SUCCESS;
}