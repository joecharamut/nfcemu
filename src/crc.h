#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CRC_A_INITIAL 0x6363

uint16_t crc16_iso14443_3_a(uint16_t crc, uint8_t byte);

#ifdef __cplusplus
}
#endif
