#include "osal.h"

#if defined(__linux__) || defined(__linux) || defined(linux)
#define _POSIX_C_SOURCE 199309L
#endif

#include <time.h>
#include <unistd.h>

void os_sleep_ms(uint32_t milliseconds) {
  struct timespec ts = {
      .tv_sec = milliseconds / 1000,
      .tv_nsec = (milliseconds % 1000) * 1000000,
  };

  nanosleep(&ts, NULL);
}
