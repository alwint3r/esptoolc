#include "esp_serial_port.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define DEFAULT_RESET_DELAY 100
#define ESP32_R0_DELAY 500
#define TOGGLE_DELAY 50

esp_error_t esp_port_open(serial_port_t* port, esp_port_config_t* config) {
  if (port == NULL || config == NULL) {
    return ESP_ERR_INVALID_PORT;
  }
  *port = open(config->port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (*port == -1) {
    perror("Error opening port");
    return ESP_ERR_PORT_OPEN;
  }

  struct termios options;
  if (tcgetattr(*port, &options) < 0) {
    close(*port);
    fprintf(stderr, "Error getting port attributes: %s\n", strerror(errno));
    return ESP_ERR_PORT_CONFIG;
  }

  speed_t baud = B115200;
  switch (config->baud_rate) {
    case 9600:
      baud = B9600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      baud = B115200;
      break;
  }

  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag |= (CLOCAL | CREAD);

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | INLCR | IGNCR | ICRNL);
  options.c_oflag &= ~OPOST;
  options.c_oflag &= ~ONLCR;

  options.c_cc[VTIME] = 10;  // No timeout
  options.c_cc[VMIN] = 0;    // Non-blocking read

  if (tcsetattr(*port, TCSANOW, &options) < 0) {
    close(*port);
    fprintf(stderr, "Error setting port attributes: %s\n", strerror(errno));
    return ESP_ERR_PORT_CONFIG;
  }

  return ESP_SUCCESS;
}

esp_error_t esp_port_close(serial_port_t port) {
  if (close(port) == -1) {
    return ESP_ERR_PORT_OPEN;
  }
  return ESP_SUCCESS;
}

static esp_error_t set_dtr(serial_port_t port, bool state) {
  int status;
  if (ioctl(port, TIOCMGET, &status) == -1) {
    return ESP_ERR_RESET_FAILED;
  }

  if (state) {
    status |= TIOCM_DTR;
  } else {
    status &= ~TIOCM_DTR;
  }

  if (ioctl(port, TIOCMSET, &status) == -1) {
    return ESP_ERR_RESET_FAILED;
  }

  return ESP_SUCCESS;
}

static esp_error_t set_rts(serial_port_t port, bool state) {
  int status;
  if (ioctl(port, TIOCMGET, &status) == -1) {
    return ESP_ERR_RESET_FAILED;
  }

  if (state) {
    status |= TIOCM_RTS;
  } else {
    status &= ~TIOCM_RTS;
  }

  if (ioctl(port, TIOCMSET, &status) == -1) {
    return ESP_ERR_RESET_FAILED;
  }

  return ESP_SUCCESS;
}

static void sleep_ms(int milliseconds) { usleep(milliseconds * 1000); }

static esp_error_t set_dtr_rts(serial_port_t port, bool dtr, bool rts) {
  int status;
  if (ioctl(port, TIOCMGET, &status) == -1) {
    return ESP_ERR_RESET_FAILED;
  }

  if (dtr) {
    status |= TIOCM_DTR;
  } else {
    status &= ~TIOCM_DTR;
  }

  if (rts) {
    status |= TIOCM_RTS;
  } else {
    status &= ~TIOCM_RTS;
  }

  if (ioctl(port, TIOCMSET, &status) == -1) {
    return ESP_ERR_RESET_FAILED;
  }
  return ESP_SUCCESS;
}

static esp_error_t perform_reset_sequence(serial_port_t port,
                                          const esp_port_config_t* config) {
  esp_error_t err;
  int reset_delay = DEFAULT_RESET_DELAY;

  if (config->esp32r0_delay) {
    reset_delay += ESP32_R0_DELAY;
  }

  if (config->reset_type == RESET_TYPE_UNIX) {
    // Unix-style reset sequence with both pins toggled simultaneously
    if ((err = set_dtr_rts(port, false, false)) != ESP_SUCCESS) return err;
    if ((err = set_dtr_rts(port, true, true)) != ESP_SUCCESS) return err;
    if ((err = set_dtr_rts(port, false, true)) != ESP_SUCCESS)
      return err;  // IO0=HIGH & EN=LOW
    sleep_ms(100);
    if ((err = set_dtr_rts(port, true, false)) != ESP_SUCCESS)
      return err;  // IO0=LOW & EN=HIGH
    sleep_ms(reset_delay);
    if ((err = set_dtr_rts(port, false, false)) != ESP_SUCCESS)
      return err;  // IO0=HIGH
    if ((err = set_dtr(port, false)) != ESP_SUCCESS)
      return err;  // Ensure IO0=HIGH
  } else if (config->reset_type == RESET_TYPE_CLASSIC) {
    // Classic reset sequence
    if ((err = set_dtr(port, false)) != ESP_SUCCESS) return err;  // IO0=HIGH
    if ((err = set_rts(port, true)) != ESP_SUCCESS)
      return err;  // EN=LOW, reset
    sleep_ms(100);
    if ((err = set_dtr(port, true)) != ESP_SUCCESS) return err;  // IO0=LOW
    if ((err = set_rts(port, false)) != ESP_SUCCESS)
      return err;  // EN=HIGH, out of reset
    sleep_ms(reset_delay);
    if ((err = set_dtr(port, false)) != ESP_SUCCESS)
      return err;  // IO0=HIGH, done
  } else {
    // Hard reset sequence
    if ((err = set_rts(port, true)) != ESP_SUCCESS) return err;  // EN=LOW
    sleep_ms(100);
    if ((err = set_rts(port, false)) != ESP_SUCCESS) return err;  // EN=HIGH
  }

  return ESP_SUCCESS;
}
esp_error_t esp_reset(serial_port_t port, esp_port_config_t* config) {
  esp_error_t err;
  int reset_delay = DEFAULT_RESET_DELAY;
  if (config->esp32r0_delay) {
    reset_delay += ESP32_R0_DELAY;
  }

  // ensure IO0 is HIGH to prevent it entering the download mode
  if ((err = set_dtr(port, false)) != ESP_SUCCESS) {
    return err;
  }

  if ((err = set_rts(port, true)) != ESP_SUCCESS) {
    return err;
  }

  sleep_ms(100);

  if ((err = set_rts(port, false)) != ESP_SUCCESS) {
    return err;
  }

  return ESP_SUCCESS;
}

esp_error_t esp_trigger_download_mode(serial_port_t port,
                                      esp_port_config_t* config) {
  if (!port || !config) {
    return ESP_ERR_INVALID_PORT;
  }

  return perform_reset_sequence(port, config);
}
