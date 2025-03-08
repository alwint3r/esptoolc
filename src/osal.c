#include "osal.h"

#include <time.h>
#include <unistd.h>

void os_sleep_ms(uint32_t milliseconds) {
  struct timespec ts = {
      .tv_sec = milliseconds / 1000,
      .tv_nsec = (milliseconds % 1000) * 1000000,
  };

  nanosleep(&ts, NULL);
}