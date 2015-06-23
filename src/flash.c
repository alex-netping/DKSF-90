/*
 * DKST 910.5 220V 1-Wire power sensor
 * Flash interface
 */
#include "flash.h"

// We use simple wear leveling algorithm.
// Config related registers are stored in the last page of flash
// Counter registers are stored in the previous page of flash (last - 1)
//
// Pages size is 1024 bytes
// The algorithm divides the page into 36 blocks
// Block size is 28 bytes: 2 byte tag + 26 bytes of config registers;
// Each block starts with a 16 bit tag constant
//
// Writing config registers to flash:
// --------------------------
// We have a "next block counter" in RAM
// Increment "next block counter"
// If "next block counter" == 36 then
//    Set "next block counter" = 0
//    Erase page
// endif
// Write block at the block area "start of page" + ("config registers size" * "next block counter");
//
// Loading config registers from flash (done at startup):
// -------------------------------------------
// bFoundTag = 0;
// Set "next block counter" = 0
// while ("next block counter" < 36)
// begin
//    read first 16 bits from current block of flash
//    if (first 16 bits of block == tag constant)
//       bFoundTag = 1;
//    else
//       break;
//    read block from flash (size equal 26 = content of registers)
//    increment "next block counter"
// end
//
// if (bFoundTag == 0)
//   return false (flash empty)
//
// decrement "next block counter"
// copy the loaded registers block into user's provided buffer of RAM registers
//
// ==========================
// Algorithm for counters stored in flash:
// There are 5 32 bit counters. Counters are stored in a separate page from configuration registers
// When counter is to be updated in flash, the idea is the following
// - 1024 byte page is used for writing records, one after another until the pade is full. Each record has:
//    - Header (16 bits), first byte = counter id, second byte = data length (2 or 4)
//    - Data, length of data is specified in the second byte of header and can be either 2 or 4 bytes
//    - If the counter value fits in 2 bytes the record will be <cnt id><2><data 2 bytes>
//    - If the counter value does not fit in 2 bytes the record will be <cnt id><4><data 4 bytes>
// - When counter needs to be written to flash, the record is formed and written to the next available unprogrammed space in the page
// - When there is no space in the page for the next record, the record is erased and the records for all registers are written from the start of page
// - The next write location is tracked in the RAM variable
//
// On startup the counters are read in from flash
// - read in the records sequencially from the start of page and the counter is set the the last read value
// - the next write location is updated as new records are read in
// - when the <cnt id> byte is FF, that's the start of free space - the next write location
//
//

#define FLASH_PAGE_SZ						1024
#define FLASH_CFG_REGISTERS_NUM_WORDS		14		// size of config registers in number of words
#define FLASH_CFG_REGISTERS_BLOCK_SZ		((FLASH_CFG_REGISTERS_NUM_WORDS * 2) + 2)		// size of block in bytes
#define FLASH_CFG_NUM_BLOCKS				34		// number of config reg blocks in a page

// Store settings in the last page of flash
// Flash starts at -0x08000000
// Flash size on this uc is 16k
#define DKST910_FLASH_ADDR_SETTINGS		0x08003c00
#define DKST910_FLASH_ADDR_COUNTERS		0x08003800

#define DKST910_CFGTAG 0x4e50



uint8_t cfgRegFlashBlockNo;
uint16_t cntRegFlashWritePtr;

// copy of counter registers
uint32_t cntRegs[5];
uint8_t cntRegChanged[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

// Init flash operations
__INLINE void flash_init(void)
{  
  	/* (1) Wait till no operation is on going */
  	/* (2) Check that the Flash is unlocked */
  	/* (3) Perform unlock sequence */
 	while ((FLASH->SR & FLASH_SR_BSY) != 0); /* (1) */  

  	if ((FLASH->CR & FLASH_CR_LOCK) != 0) /* (2) */
  	{
    	FLASH->KEYR = FLASH_FKEY1; /* (3) */
    	FLASH->KEYR = FLASH_FKEY2;
  	}
}

// Erase a page of flash
static void flash_erase(uint32_t page_addr)
{   
  	/* (1) Set the PER bit in the FLASH_CR register to enable page erasing */
  	/* (2) Program the FLASH_AR register to select a page to erase */
  	/* (3) Set the STRT bit in the FLASH_CR register to start the erasing */
  	/* (4) Wait until the BSY bit is reset in the FLASH_SR register */
  	/* (5) Check the EOP flag in the FLASH_SR register */
  	/* (6) Clear EOP flag by software by writing EOP at 1 */
  	/* (7) Reset the PER Bit to disable the page erase */
  	FLASH->CR |= FLASH_CR_PER; /* (1) */    
  	FLASH->AR =  page_addr; /* (2) */    
  	FLASH->CR |= FLASH_CR_STRT; /* (3) */    
  	while ((FLASH->SR & FLASH_SR_BSY) != 0) ; /* (4) */ 

  	if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (5) */
  	{  
    	FLASH->SR |= FLASH_SR_EOP; /* (6)*/
  	}
	
  	/* Manage the error cases */
  	else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) /* Check Write protection error */
    	FLASH->SR |= FLASH_SR_WRPERR; /* Clear the flag by software by writing it at 1*/

  	FLASH->CR &= ~FLASH_CR_PER; /* (7) */
}

// Program 16 bit word to flash
static void flash_word16_prog(uint32_t flash_addr, uint16_t data)
{	
  	/* (1) Set the PG bit in the FLASH_CR register to enable programming */
  	/* (2) Perform the data write (half-word) at the desired address */
  	/* (3) Wait until the BSY bit is reset in the FLASH_SR register */
  	/* (4) Check the EOP flag in the FLASH_SR register */
  	/* (5) clear it by software by writing it at 1 */
  	/* (6) Reset the PG Bit to disable programming */
  	FLASH->CR |= FLASH_CR_PG; /* (1) */

	*(__IO uint16_t*)(flash_addr) = data; /* (2) */
	
  	while ((FLASH->SR & FLASH_SR_BSY) != 0) ; /* (3) */

  	if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (4) */
    	FLASH->SR |= FLASH_SR_EOP; /* (5) */
  	else if ((FLASH->SR & FLASH_SR_PGERR) != 0) /* Check Programming error */
    	FLASH->SR |= FLASH_SR_PGERR; /* Clear it by software by writing EOP at 1*/
  	else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) /* Check write protection */
    	FLASH->SR |= FLASH_SR_WRPERR; /* Clear it by software by writing it at 1*/
	
  	FLASH->CR &= ~FLASH_CR_PG; /* (6) */
}

// Load block of data from flash
static void flash_read_block(uint32_t flash_addr, uint16_t *wToAddr, uint16_t NumWords)
{
	while (NumWords--)
	{
		*wToAddr = *(__IO uint16_t*)(flash_addr);
		flash_addr += 2;
		wToAddr++;	
	}	
}

// Write block of data to flash
static void flash_write_block(uint32_t flash_addr, uint16_t *wFromAddr, uint16_t NumWords)
{
	while (NumWords--)
	{
		flash_word16_prog(flash_addr, *wFromAddr);
		flash_addr += 2;
		wFromAddr++;	
	}
}

// Load registers from flash
// wToAddr - address of registers RAM image
// returns 1 = registers loaded
// returns 0 = flash empty (has not been programmed)
int FlashLoadCfgRegisters(uint16_t *wToAddr)
{	
	uint16_t blockBuf[FLASH_CFG_REGISTERS_NUM_WORDS];
	uint32_t flash_addr = DKST910_FLASH_ADDR_SETTINGS;
	uint8_t bFoundTag = 0;
	uint16_t tag, i;

	flash_init();
	
	cfgRegFlashBlockNo = 0;
	while (cfgRegFlashBlockNo < FLASH_CFG_NUM_BLOCKS)
	{
		tag = *(__IO uint16_t*)(flash_addr);
		if (tag != DKST910_CFGTAG)
			break;
		
		bFoundTag = 1;
		flash_addr += 2;

		flash_read_block(flash_addr, &blockBuf[0], FLASH_CFG_REGISTERS_NUM_WORDS);
		flash_addr += (FLASH_CFG_REGISTERS_NUM_WORDS * 2);
		cfgRegFlashBlockNo++;
	}

	if (!bFoundTag)
		return 0;

	for (i=0; i<FLASH_CFG_REGISTERS_NUM_WORDS; i++)
		wToAddr[i] = blockBuf[i];

	return 1;
}

// Write config registers to flash
// wFromAddr - address of registers RAM image
void FlashWriteCfgRegisters(uint16_t *wFromAddr)
{
	uint32_t flash_addr;
	
	if (cfgRegFlashBlockNo == FLASH_CFG_NUM_BLOCKS)
	{
		cfgRegFlashBlockNo = 0;
		flash_erase(DKST910_FLASH_ADDR_SETTINGS);
	}

	flash_addr = DKST910_FLASH_ADDR_SETTINGS + (cfgRegFlashBlockNo * FLASH_CFG_REGISTERS_BLOCK_SZ);

	flash_word16_prog(flash_addr, DKST910_CFGTAG);
	flash_addr += 2;

	flash_write_block(flash_addr, wFromAddr, FLASH_CFG_REGISTERS_NUM_WORDS);	

	cfgRegFlashBlockNo++;
}

// Load counter registers from flash
// wToAddr - address of the first RAM image counter register
// the counter registers in RAM are contigeous 5 32 bit registers in specific order
// It is assumed flash_init was already called, as FlashLoadCfgRegisters is called first
void FlashLoadCounterRegisters(uint32_t *wToAddr)
{
	uint32_t flash_addr = DKST910_FLASH_ADDR_COUNTERS;
	uint8_t cntId, dataLen;
	uint16_t hdr, cntVal16;
	uint32_t cntVal32;
	
	cntRegFlashWritePtr = 0;
	while (cntRegFlashWritePtr < FLASH_PAGE_SZ)
	{
		hdr = *(__IO uint16_t*)(flash_addr);
		cntId = (uint8_t)((hdr >> 8) & 0xff);
		dataLen = (uint8_t)(hdr & 0xff);

		if ((cntId == 0xff) || (cntId > DKST910_FLASH_CNT_BLKOUT))
			break;

		flash_addr += 2;
		cntRegFlashWritePtr += 2;

		if (dataLen == 2)
		{
			cntVal16 = *(__IO uint16_t*)(flash_addr);
			cntVal32 = (uint32_t)cntVal16;
			flash_addr += 2;
			cntRegFlashWritePtr += 2;
		}
		else if (dataLen == 4)
		{
			cntVal16 = *(__IO uint16_t*)(flash_addr);
			cntVal32 = (uint32_t)(cntVal16 << 16) & 0xffff0000;
			flash_addr += 2;
			cntVal16 = *(__IO uint16_t*)(flash_addr);
			cntVal32 |= (uint32_t)(cntVal16 & 0x0000ffff); 
			flash_addr += 2;
			cntRegFlashWritePtr += 4;
		}
		// something wrong, should be 2 or 4
		else
		{
			break;
		}

		wToAddr[cntId] = cntVal32;
		cntRegs[cntId] = cntVal32;
	}
}

// Notifies the flash module that specific counter changed
void FlashCounterRegisterNotifyChange(uint8_t regId, uint32_t cntData)
{
	cntRegs[regId] = cntData;
	cntRegChanged[regId] = 0x01;	
}

// Write counter register to flash
static void flash_write_counter(uint8_t regId, uint32_t cntData)
{
	uint32_t flash_addr = DKST910_FLASH_ADDR_COUNTERS;
	uint8_t dataLen;
	uint16_t hdr;

	dataLen = (cntData > 0xffff) ? 4 : 2;

	hdr = (uint16_t)(regId << 8) & 0xff00;
	hdr |= (uint16_t)(dataLen & 0xff);

	flash_addr += cntRegFlashWritePtr;

	flash_word16_prog(flash_addr, hdr);
	flash_addr += 2;
	
	flash_word16_prog(flash_addr, cntData);
	cntRegFlashWritePtr += (2 + dataLen);	
}

// Called at a timed interval when it is time to update in flash any registers that have changed
void FlashCounterRegistersUpdate(void)
{
	uint8_t regId;
	uint8_t dataLen;
	uint16_t spaceRequired = 0;

	// find space required to write all changed counters
	for (regId = 0; regId < DKST910_NUM_COUNTERS; regId++)
	{
		if (cntRegChanged[regId])
		{
			dataLen = (cntRegs[regId] > 0xffff) ? 4 : 2;
			spaceRequired += (dataLen + 2);
		}
	}

	// if not enough space to the end of page to write all changed counters
	if ((cntRegFlashWritePtr + spaceRequired) > FLASH_PAGE_SZ)
	{
		cntRegFlashWritePtr = 0;
		flash_erase(DKST910_FLASH_ADDR_COUNTERS);
		// all counter must be written to flash now

		for (regId=0; regId < DKST910_NUM_COUNTERS; regId++)
			cntRegChanged[regId] = 0x01;
	}

	// write each changed counter to flash
	for (regId = 0; regId < DKST910_NUM_COUNTERS; regId++)
	{
		if (cntRegChanged[regId])
		{
			cntRegChanged[regId] = 0x00;
			flash_write_counter(regId, cntRegs[regId]);
		}
	}
}
