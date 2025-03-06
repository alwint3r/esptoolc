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

// get the first 8 bytes
// determine the variable length
// consume for the rest of the length
// repeat

esp_error_t sync_chip(serial_port_t port) {
  uint8_t command_out_buf[128];
  size_t command_out_len = esp_command_sync_encoded(command_out_buf);
  esp_error_t err;
  err = esp_write(port, command_out_buf, command_out_len);
  if (err != ESP_SUCCESS) {
    fprintf(stderr, "Failed to write command to ESP chip: %d\n", err);
    return err;
  }

  printf("Sent sync command to ESP chip. (%zu bytes)\n", command_out_len);
  dump_hex(command_out_buf, command_out_len);

  printf("Waiting for response...\n");

  uint8_t response_buf[128];
  size_t total_read_length = 0;
  for (int i = 0; i < 100; i++) {
    size_t read_length = 0;
    err = esp_read_timeout(port, response_buf + total_read_length,
                           sizeof(response_buf) - total_read_length, 100,
                           &read_length);
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

    total_read_length += read_length;
  }

  printf("Decoding response...\n");
  printf("Total read length: %zu\n", total_read_length);
  dump_hex(response_buf, total_read_length);

  size_t message_count = 0;
  bool in_escape = false;
  bool header_start = false;
  for (size_t i = 0; i < total_read_length; i++) {
    if (response_buf[i] == 0xDB) {
      in_escape = true;
      continue;
    }

    if (in_escape) {
      in_escape = false;
    }

    if (response_buf[i] == 0xC0) {
      if (header_start) {
        header_start = false;
        message_count++;
      } else {
        header_start = true;
      }
      continue;
    }
  }

  if (message_count == 0) {
    fprintf(stderr, "No messages found in response\n");
    return ESP_ERR_INVALID_RESPONSE;
  }
  printf("Found %zu messages in response\n", message_count);

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

  usleep(1 * 1000 * 1000);  // Wait for 5 seconds
  read_thread_running = false;
  pthread_join(read_thread, NULL);

  printf("Download mode triggered successfully.\n");
  get_serial_lines(port);
  sync_chip(port);
  printf("Press Ctrl+C to exit.\n");
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
