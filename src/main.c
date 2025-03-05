#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_serial_port.h"

volatile bool read_thread_running = true;
void *serial_read_thread(void *arg);
void signal_handler(int sig);

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

  pthread_t read_thread;
  if (pthread_create(&read_thread, NULL, serial_read_thread, &port) != 0) {
    fprintf(stderr, "Failed to create read thread\n");
    esp_port_close(port);
    return 1;
  }

  esp_reset(port, &config);

  usleep(3 * 1000 * 1000);  // Wait for 10 seconds

  if ((err = esp_trigger_download_mode(port, &config)) != ESP_SUCCESS) {
    fprintf(stderr, "Failed to trigger download mode: %d\n", err);
    read_thread_running = false;
    pthread_join(read_thread, NULL);
    esp_port_close(port);
    return 1;
  }

  printf("Download mode triggered successfully.\n");
  printf("Press Ctrl+C to exit.\n");

  pthread_join(read_thread, NULL);
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