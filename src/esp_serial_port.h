#pragma once

#include <stdbool.h>

typedef int serial_port_t;

typedef enum {
    ESP_SUCCESS = 0,
    ESP_ERR_INVALID_PORT = -1,
    ESP_ERR_PORT_OPEN = -2,
    ESP_ERR_PORT_CONFIG = -3,
    ESP_ERR_RESET_FAILED = -4,
} esp_error_t;

typedef enum {
    RESET_TYPE_CLASSIC = 0,
    RESET_TYPE_UNIX,
    RESET_TYPE_HARD,
} reset_type_t;

typedef struct {
    const char* port_name;
    int baud_rate;
    bool esp32r0_delay;
    reset_type_t reset_type;
} esp_port_config_t;

esp_error_t esp_port_open(serial_port_t* port, esp_port_config_t* config);
esp_error_t esp_port_close(serial_port_t port);
esp_error_t esp_trigger_download_mode(serial_port_t port, esp_port_config_t* config);
esp_error_t esp_reset(serial_port_t port, esp_port_config_t* config);
