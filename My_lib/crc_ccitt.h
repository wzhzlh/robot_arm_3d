#ifndef __CRC_CCITT_H
#define __CRC_CCITT_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

#ifdef __cplusplus
}
#endif

#endif