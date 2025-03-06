#pragma once

#include <stdint.h>
#include <stdlib.h>

size_t esp_command_slip_encode(uint8_t *dest, const uint8_t *src, size_t src_len);
size_t esp_command_slip_decode(uint8_t *dest, const uint8_t *src, size_t src_len);
uint8_t esp_command_checksum(uint8_t *data, size_t length);