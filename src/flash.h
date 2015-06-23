/*
 * DKST 910.5 220V 1-Wire power sensor
 * Flash interface
 */
#ifndef _DKST910_FLASH_H
#define _DKST910_FLASH_H

#include "stm32f0xx.h"

#define DKST910_FLASH_CNT1_UV		0x00
#define DKST910_FLASH_CNT1_OV		0x01
#define DKST910_FLASH_CNT2_UV		0x02
#define DKST910_FLASH_CNT2_OV		0x03
#define DKST910_FLASH_CNT_BLKOUT	0x04

#define DKST910_NUM_COUNTERS		5		// number of counter registers

// Load configuration registers from flash
// wToAddr - address of registers RAM image
// returns 1 = registers loaded
// returns 0 = flash empty (has not been programmed)
extern int FlashLoadCfgRegisters(uint16_t *wToAddr);

// Write config registers to flash
// wFromAddr - address of registers RAM image
extern void FlashWriteCfgRegisters(uint16_t *wFromAddr);

// Load counter registers from flash
// wToAddr - address of the first RAM image counter register
// the counter registers in RAM are contigeous 5 32 bit registers in specific order
// It is assumed flash_init was already called, as FlashLoadCfgRegisters is called first
extern void FlashLoadCounterRegisters(uint32_t *wToAddr);

// Notifies the flash module that specific counter changed
extern void FlashCounterRegisterNotifyChange(uint8_t regId, uint32_t cntData);

// Called at a timed interval when it is time to update in flash any registers that have changed
extern void FlashCounterRegistersUpdate(void);

#endif
