/*
 * DKST 910.5 220V 1-Wire power sensor
 * Flash interface
 */
#ifndef _DKST910_CRC_H
#define _DKST910_CRC_H

#include "stm32f0xx.h"

__INLINE uint16_t crc_16_update(uint16_t crc, uint8_t data);

extern unsigned char calc_crc(unsigned char *code, unsigned char code_len);

#endif // _DKST910_CRC_H
