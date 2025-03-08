#include "esp_serial_port.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "os_hal.h"

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

esp_error_t esp_set_dtr(serial_port_t port, bool state) {
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

esp_error_t esp_set_rts(serial_port_t port, bool state) {
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

esp_error_t esp_set_dtr_rts(serial_port_t port, bool dtr, bool rts) {
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

esp_error_t esp_read_timeout(serial_port_t port, uint8_t* buffer, size_t size,
                             int timeout_ms, size_t* out_size) {
  if (port < 0) {
    return ESP_ERR_INVALID_PORT;
  }

  if (buffer == NULL || size == 0 || out_size == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  fd_set read_fds;
  struct timeval tv;
  int bytes_read = 0;

  FD_ZERO(&read_fds);
  FD_SET(port, &read_fds);

  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int result = select(port + 1, &read_fds, NULL, NULL, &tv);
  if (result < 0) {
    return ESP_ERR_READ_FAILED;
  } else if (result == 0) {
    return ESP_ERR_TIMEOUT;  // Timeout occurred
  }

  bytes_read = read(port, buffer, size);
  if (bytes_read < 0) {
    return ESP_ERR_READ_FAILED;
  }

  *out_size = bytes_read;

  return ESP_SUCCESS;
}

esp_error_t esp_write(serial_port_t port, const uint8_t* data, size_t size) {
  if (port < 0) {
    return ESP_ERR_INVALID_PORT;
  }

  if (data == NULL || size == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  ssize_t bytes_written = write(port, data, size);
  if (bytes_written < 0) {
    return ESP_ERR_WRITE_FAILED;
  }

  return ESP_SUCCESS;
}

esp_error_t esp_discard_input(serial_port_t port) {
  int ierr;
  int bytes_available;
  esp_error_t err;
  if ((ierr = ioctl(port, FIONREAD, &bytes_available)) < 0) {
    err = ESP_ERR_READ_FAILED;
    return err;
  }

  if (bytes_available > 0) {
    uint8_t buffer[128];
    ssize_t bytes_read;
    while ((bytes_read = read(port, buffer, sizeof(buffer))) > 0) {
      // Disadcard the data
    }
  }

  return ESP_SUCCESS;
}