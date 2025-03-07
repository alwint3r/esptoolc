#include "slip_reader.h"

#include <stdbool.h>
#include <stdio.h>

#include "slip_def.h"

void slip_reader_init(slip_reader_t *reader, uint8_t *output_buf,
                      size_t capacity) {
  reader->buf = output_buf;
  reader->length = 0;
  reader->capacity = capacity;
  reader->state = SLIP_READER_START;
  reader->is_in_escape = false;
  reader->processed_count = 0;
}

void slip_reader_reset(slip_reader_t *reader) {
  reader->length = 0;
  reader->state = SLIP_READER_START;
  reader->is_in_escape = false;
  reader->processed_count = 0;
}

bool slip_reader_data_ready(slip_reader_t *reader) {
  return (reader->state == SLIP_READER_END);
}

slip_reader_state_t slip_reader_process_byte(slip_reader_t *reader,
                                             uint8_t byte) {
  switch (reader->state) {
    case SLIP_READER_START:
      if (byte == SLIP_END) {
        reader->state = SLIP_READER_DATA;
        reader->length = 0;
      }
      reader->processed_count++;
      break;

    case SLIP_READER_DATA:
      if (reader->is_in_escape) {
        if (byte == SLIP_ESC_END) {
          reader->buf[reader->length++] = SLIP_END;
        } else if (byte == SLIP_ESC_ESC) {
          reader->buf[reader->length++] = SLIP_ESC;
        }
        reader->is_in_escape = false;
      } else {
        if (byte == SLIP_END) {
          reader->state = SLIP_READER_END;
        } else if (byte == SLIP_ESC) {
          reader->is_in_escape = true;
        } else {
          if (reader->length < reader->capacity) {
            reader->buf[reader->length++] = byte;
          }
        }
      }
      reader->processed_count++;
      break;

    case SLIP_READER_END:
      break;
  }
  return reader->state;
}
