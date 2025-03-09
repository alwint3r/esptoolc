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
#include "osal.h"
#include "slip_reader.h"
#include "slip_writer.h"

#define START_PORT_READER_THREAD 0

static void dump_hex(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    printf("%02X ", data[i]);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

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

  err = esp_chip_sync(port);
  if (err != ESP_SUCCESS) {
    fprintf(stderr, "Failed to sync with ESP chip: %d\n", err);
    esp_port_close(port);
    return 1;
  }
  printf("ESP chip synced successfully.\n");

  esp_chip_type_t chip_type;
  err = esp_chip_get_type(port, &chip_type);
  if (err != ESP_SUCCESS) {
    fprintf(stderr, "Failed to get chip type: %d\n", err);
    esp_port_close(port);
    return 1;
  }

  printf("Found chip type: %s\n", esp_chip_type_str(chip_type));

  esp_chip_hard_reset(port);

  esp_port_close(port);
  return 0;
}
