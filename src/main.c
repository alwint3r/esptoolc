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
#include "esp32/esp32_chip.h"
#include "esp_chip.h"
#include "esp_serial_port.h"
#include "osal.h"
#include "slip_reader.h"
#include "slip_writer.h"

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

  uint32_t baud = (uint32_t)atoll(baud_str);
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

  if (chip_type == ESP_CHIP_ESP32) {
    esp32_chip_desc_t chip_desc;
    err = esp32_get_chip_desc(port, &chip_desc);
    if (err != ESP_SUCCESS) {
      fprintf(stderr, "Failed to get chip description: %d\n", err);
      esp_port_close(port);
      return 1;
    }

    printf("Chip is %s (revision v%u.%u)\n",
           esp32_chip_name_str(chip_desc.name), chip_desc.major_rev,
           chip_desc.minor_rev);

    uint8_t xtal_freq;
    err = esp32_get_crystal_freq(port, (int32_t)baud, &xtal_freq);
    if (err != ESP_SUCCESS) {
      fprintf(stderr, "Failed to get crystal frequency: %d\n", err);
      esp_port_close(port);
      return 1;
    }

    printf("Crystal is %u MHz\n", xtal_freq);

    uint8_t mac[6];
    err = esp32_read_mac(port, mac);
    if (err != ESP_SUCCESS) {
      fprintf(stderr, "Failed to read MAC address: %d\n", err);
      esp_port_close(port);
      return 1;
    }
    printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);
  }

  esp_chip_hard_reset(port);

  esp_port_close(port);
  return 0;
}
