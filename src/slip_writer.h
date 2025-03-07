#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

/**
 * @brief Encode a byte array.
 * This function assumes that the destination buffer is large enough to hold the
 * encoded data. It does not check for buffer overflows.
 * @param dest Pointer to the destination buffer
 * @param src Pointer to the source byte array
 * @param src_len Length of the source byte array
 * @return Number of bytes written or -1 on error
 */
ssize_t slip_write(uint8_t* dest, const uint8_t* src, size_t src_len);
