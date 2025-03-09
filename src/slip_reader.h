#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  SLIP_READER_START,
  SLIP_READER_DATA,
  SLIP_READER_END,
} slip_reader_state_t;

typedef struct {
  // data
  uint8_t *buf;
  size_t length;
  size_t capacity;

  slip_reader_state_t state;
  size_t processed_count;
  bool is_in_escape;
} slip_reader_t;

void slip_reader_init(slip_reader_t *reader, uint8_t *output_buf,
                      size_t capacity);
void slip_reader_reset(slip_reader_t *reader);
bool slip_reader_data_ready(slip_reader_t *reader);
slip_reader_state_t slip_reader_process_byte(slip_reader_t *reader,
                                             uint8_t byte);
