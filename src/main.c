#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "commands/esp_command.h"
#include "commands/esp_command_sync.h"
#include "esp_serial_port.h"
#include "os_hal.h"
#include "slip_reader.h"

volatile bool read_thread_running = true;
void *serial_read_thread(void *arg);
void signal_handler(int sig);

int get_serial_lines(int fd) {
  int status;
  ioctl(fd, TIOCMGET, &status);

  printf("DTR: %s\n", (status & TIOCM_DTR) ? "HIGH(0)" : "LOW(1)");
  printf("RTS: %s\n", (status & TIOCM_RTS) ? "LOW(1)" : "HIGH(0)");

  return status;
}

void dump_hex(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    printf("%02X ", data[i]);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
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
  size_t total_read_length = 0;
  slip_reader_t slip_reader;

  uint8_t response_count = 0;
  uint8_t max_response_count = 8;
  slip_reader_init(&slip_reader, slip_reader_buf, sizeof(slip_reader_buf));

  for (int i = 0; i < 100; i++) {
    size_t read_length = 0;
    err = esp_read_timeout(port, response_buf, 24, 100, &read_length);
    if (err != ESP_SUCCESS) {
      if (err == ESP_ERR_TIMEOUT) {
        if (total_read_length == 0) {
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

    total_read_length += read_length;
  }

  if (response_count == 0) {
    fprintf(stderr, "No messages found in response\n");
    return ESP_ERR_INVALID_RESPONSE;
  }
  printf("Found %zu messages in response\n", response_count);

  return ESP_SUCCESS;
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
      .reset_type = RESET_TYPE_UNIX,
  };

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  esp_error_t err;
  if ((err = esp_port_open(&port, &config)) != ESP_SUCCESS) {
    fprintf(stderr, "Failed to open port: %d\n", err);
    return 1;
  }

  esp_reset(port, &config);

  os_sleep_ms(3000);

  if ((err = esp_enter_download_mode(port, &config)) != ESP_SUCCESS) {
    fprintf(stderr, "Failed to trigger download mode: %d\n", err);
    esp_port_close(port);
    return 1;
  }

  os_sleep_ms(5000);

  pthread_t read_thread;
  if (pthread_create(&read_thread, NULL, serial_read_thread, &port) != 0) {
    fprintf(stderr, "Failed to create read thread\n");
    esp_port_close(port);
    return 1;
  }

  tcflush(port, TCIFLUSH);

  usleep(1 * 1000 * 1000);  // Wait for 5 seconds
  read_thread_running = false;
  pthread_join(read_thread, NULL);

  printf("Download mode triggered successfully.\n");
  get_serial_lines(port);
  sync_chip(port);
  esp_port_close(port);
  return 0;
}

void *serial_read_thread(void *arg) {
  serial_port_t *port = (serial_port_t *)arg;
  char buffer[256];
  memset(buffer, 0, sizeof(buffer));
  while (read_thread_running) {
    ssize_t bytes_read = read(*port, buffer, sizeof(buffer));
    if (bytes_read > 0) {
      fwrite(buffer, 1, bytes_read, stdout);
      fflush(stdout);
    }
    usleep(10000);
  }
  return NULL;
}

void signal_handler(int sig) {
  read_thread_running = false;
  printf("\nSignal %d received, stopping read thread...\n", sig);
}
