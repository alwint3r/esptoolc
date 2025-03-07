#include "slip_writer.h"

#include "slip_def.h"

ssize_t slip_write(uint8_t *dest, const uint8_t *src, size_t src_len) {
  if (!dest || !src || src_len == 0) {
    return -1;
  }
  size_t written = 0;
  dest[written++] = SLIP_END;

  for (size_t i = 0; i < src_len; i++) {
    if (src[i] == SLIP_END) {
      dest[written++] = SLIP_ESC;
      dest[written++] = SLIP_ESC_END;
    } else if (src[i] == SLIP_ESC) {
      dest[written++] = SLIP_ESC;
      dest[written++] = SLIP_ESC_ESC;
    } else {
      dest[written++] = src[i];
    }
  }
  dest[written++] = SLIP_END;
  return written;
}