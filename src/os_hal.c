#include "os_hal.h"

#include <unistd.h>

void os_sleep_ms(uint32_t milliseconds) { usleep(milliseconds * 1000); }