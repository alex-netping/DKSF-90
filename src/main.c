/*
 * DKST 910.5 220V 1-Wire power sensor
 */

#include <string.h>
#include "stm32f0xx.h"
#include "dkst910_defs.h"
#include "flash.h"
#include "crc.h"

static volatile uint8_t opFlags = F_MEASURMENT_ALLOWED;

// ADC samples for the measured period
// At present this buffer is used only for debugging
// Voltage and frequency are measured on the fly by processing each sample.
uint16_t adcSamples[NUM_ADC_SAMPLES];
uint16_t sampleCount;
uint16_t halfPeriodSampleCount;

// Soft registers memory
volatile unsigned char Dkst910RegMem[DKST910_REGMEM_SZ];

// Statistics block, full version
unsigned char Dkst910FullStats[DKST910_FULL_STATS_BLOCK_SZ];
unsigned char Dkst910ShortStats[DKST910_SHORT_STATS_BLOCK_SZ];

volatile int ow_state = OW_ST_IDLE;
volatile int ow_writeEndState = OW_ST_WR_END;
volatile int ow_readSampleState = OW_ST_RD_SAMPLE;
volatile int owCmdState = OW_CMDST_ROM;
static uint16_t ow_start_usec = 0;
volatile int ow_bitCount = 0;
volatile int ow_byteCount = 0;
volatile unsigned char ow_curByte;
unsigned char owData = 0x00;
static unsigned char ow_romCode[8] = {0xAC, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0xc7};
unsigned long long int ow_InitRomCode;
unsigned long long int ow_SearchRomCode;

volatile unsigned char *pOwTxData;
static volatile int funcDataLen = 0;
static volatile int funcRespLen = 0;

unsigned char owRegAddr;
unsigned char owCmdCode;
unsigned char owDataLen;
unsigned char owCRC16Buf[2];
unsigned char owDataBuf[DKST910_REGMEM_SZ];

uint16_t owCrc16;
uint16_t owRxCrc16;

dkst910_thres_state_t thresState[2] = 
{
	{CONDITION_NORMAL, 0, 0},
	{CONDITION_NORMAL, 0, 0},
};

uint16_t cntUpdateMs = 0;

// delay using timer TIM3
static void delayMicroseconds(uint16_t delay_us)
{
	uint16_t usecStartVal, usecCurrVal;
	uint16_t usecDiff = 0;

	// Overhead of this function is ~10 usec
	delay_us -= 10;

	usecStartVal = TIM3->CNT;
	while (usecDiff < delay_us)
	{
		usecCurrVal = TIM3->CNT;
		usecDiff = (usecCurrVal - usecStartVal);
	}
}

// Configure GPIO
static void gpio_configure(GPIO_TypeDef *pIO, int pinNo, int otype, int mode, int pullMode)
{
	pIO->MODER &= ~((uint32_t)(GPIO_MODE_MASK << (pinNo * 2)));
	pIO->MODER |= (uint32_t)(mode << (pinNo * 2));

	if (otype == GPIO_OTYPE_PUSHPULL)
		pIO->OTYPER &= ~(uint16_t)(1 << pinNo);
	else
		pIO->OTYPER |= (uint16_t)(1 << pinNo);

	pIO->PUPDR &= ~((uint32_t)(GPIO_PUPDR_MASK << (pinNo * 2)));
	pIO->PUPDR |= (uint32_t)(pullMode << (pinNo * 2));

	pIO->OSPEEDR &= ~((uint32_t)(GPIO_SPEED_MASK << (pinNo * 2)));
	pIO->OSPEEDR |= (uint32_t)(GPIO_SPEED_HIGH << (pinNo * 2));
}

// Get serial number, generated from unique device electronic signature registers (96 bits)
// Concatenated to 6 byte hex string using a hash function
static void get_device_id(unsigned char *idBuf)
{
	unsigned char serialRaw[12];
	unsigned int hash = 0;
	int i;

	*(unsigned int *)&serialRaw[0] = *(__IO uint32_t*)(0x1FFFF7AC);
	*(unsigned int *)&serialRaw[4] = *(__IO uint32_t*)(0x1FFFF7B0);
	*(unsigned int *)&serialRaw[8] = *(__IO uint32_t*)(0x1FFFF7B4);

	// Ly hash function
	for(i=0; i<12; i++)
        hash = (hash * 1664525) + (unsigned char)(serialRaw[i]) + 1013904223;

	for (i=0; i<6; i++)
	{
		idBuf[i++] = (unsigned char)(hash & 0xff);
		hash = (hash >> 8);
	}
}

// General purpose (TIM3) Timer interrupt
void TIM3_IRQHandler(void)
{
	TIM3->SR = 0;
	
	switch (ow_state)
	{
	case OW_ST_WR_END:
		// if current bit sent was '0'
		if (!(ow_curByte & 0x01))
		{
			// Release line 
			GPIOA->MODER &= ~0x00140000;	// configure PA9,10 as input				
		}

		// stop timer counter
		TIM3->CR1 &= ~TIM_CR1_CEN;

		ow_curByte >>= 1; 
		ow_bitCount++;

		// if transmitted less than 8 bits, move to transmitting next bit
		if (ow_bitCount < 8)
		{
			ow_state = OW_ST_WR_TIMESLOT;
			break;
		}

		ow_bitCount = 0;

		// 8 bits transmitted
		ow_byteCount++;

		// if responding to a ROM command
		if (owCmdState == OW_CMDST_ROM)
		{
			// If responding to READ ROM command
			if (owData == OW_CMD_READROM)
			{
				// if ROM code fully sent (8 bytes, 64 bits)
				if (ow_byteCount == 8)
				{
					EXTI->RTSR |= 0x00000400;	// enable rising edge (falling edge already on)
					ow_state = OW_ST_IDLE;

					RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
					RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
					TIM3->PSC = 7;
					TIM3->CR1 |= TIM_CR1_CEN;
					
					opFlags = F_MEASURMENT_ALLOWED;					
				}
				// If ROM code not fully sent, move to sending next bit of the next byte
				else
				{
					ow_curByte = ow_romCode[ow_byteCount];	
					ow_state = OW_ST_WR_TIMESLOT;
				}
			}			
		}
		// Responding to a function command
		else 
		{
			// Response to a function command fully sent
			if (ow_byteCount == owDataLen)
			{				
				if ((owCmdCode == OW_CMD_READ_REG) || 
					(owCmdCode == OW_CMD_READ_STATS) || 
					(owCmdCode == OW_CMD_READ_SHORTSTATS))
				{
					ow_byteCount = 0;
					owCRC16Buf[0] = (uint8_t)((owCrc16 >> 8) & 0xff);
					owCRC16Buf[1] = (uint8_t)(owCrc16 & 0xff);
					ow_curByte = owCRC16Buf[ow_byteCount];	
					ow_writeEndState = OW_ST_WR_END_CRC;
					ow_state = OW_ST_WR_TIMESLOT;
					break;
				}

				EXTI->RTSR |= 0x00000400;	// enable rising edge (falling edge already on)
				ow_state = OW_ST_IDLE;

				RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
				RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
				TIM3->PSC = 7;
				TIM3->CR1 |= TIM_CR1_CEN;

				opFlags &= ~F_MEASURMENT_RUNNING;
				opFlags |= F_MEASURMENT_ALLOWED;	
			}
			// If response data not fully sent, move to sending next bit of the next byte
			else
			{
				ow_curByte = pOwTxData[ow_byteCount];
				owCrc16 = crc_16_update(owCrc16, ow_curByte);
				ow_state = OW_ST_WR_TIMESLOT;
			}
		}
		break;

	case OW_ST_WR_END_CRC:
		// if current bit sent was '0'
		if (!(ow_curByte & 0x01))
		{
			// Release line 
			GPIOA->MODER &= ~0x00140000;	// configure PA9,10 as input				
		}

		// stop timer counter
		TIM3->CR1 &= ~TIM_CR1_CEN;

		ow_curByte >>= 1; 
		ow_bitCount++;

		// if transmitted less than 8 bits, move to transmitting next bit
		if (ow_bitCount < 8)
		{
			ow_state = OW_ST_WR_TIMESLOT;
			break;
		}

		ow_bitCount = 0;

		// 8 bits transmitted
		ow_byteCount++;

		if (ow_byteCount == 2)
		{
			EXTI->RTSR |= 0x00000400;	// enable rising edge (falling edge already on)
			ow_state = OW_ST_IDLE;

			// Restart timer counter in free running mode
			// Reset and re-init general purpose timer TIM3
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
			TIM3->PSC = 7;
			TIM3->CR1 |= TIM_CR1_CEN;

			opFlags = F_MEASURMENT_ALLOWED;
		}
		else
		{
			ow_curByte = owCRC16Buf[ow_byteCount];	
			ow_state = OW_ST_WR_TIMESLOT;
		}
		break;
		
	case OW_ST_RD_SAMPLE:
		// sample the line and store next bit
		if (GPIOA->IDR & 0x0400)
			owData |= 0x80;
		else
			owData &= ~0x80;

		// stop timer counter
		TIM3->CR1 &= ~TIM_CR1_CEN;

		// if not received full 8 bits, move on to receive next bit
		ow_bitCount++;
		if (ow_bitCount < 8)
		{
			owData >>= 1;
			ow_state = OW_ST_RD_TIMESLOT;
		}
		// received full 8 bits
		else
		{
			ow_bitCount = 0;
			
			// if expecting a function command
			if (owCmdState == OW_CMDST_FUNC)
			{
				// read register opcode, setup to transmit register data
				owCmdCode = owData;
				if ((owCmdCode == OW_CMD_READ_REG) || (owCmdCode == OW_CMD_WRITE_REG))
				{
					owCmdState = OW_CMDST_FADDR;
					ow_state = OW_ST_RD_TIMESLOT;
					break;
				}
				else if (owCmdCode == OW_CMD_READ_STATS)
				{
					owCrc16 = 0x0000;
					ow_byteCount = 0;
					owDataLen = DKST910_FULL_STATS_BLOCK_SZ;
					pOwTxData = Dkst910FullStats;
					ow_curByte = pOwTxData[ow_byteCount];
					ow_state = OW_ST_WR_TIMESLOT;
					break;
				}
				else if (owCmdCode == OW_CMD_READ_SHORTSTATS)
				{
					owCrc16 = 0x0000;
					ow_byteCount = 0;
					owDataLen = DKST910_SHORT_STATS_BLOCK_SZ;
					pOwTxData = Dkst910ShortStats;
					ow_curByte = pOwTxData[ow_byteCount];
					ow_state = OW_ST_WR_TIMESLOT;
					break;
				}
				else if (owCmdCode == OW_CMD_SYSRESET)
				{
					owCmdState = OW_CMDST_FMAGIC;
					ow_byteCount = 0;
					ow_state = OW_ST_RD_TIMESLOT;
				}
				// unsupported opcode, back to idle state
				else
				{
					EXTI->RTSR |= 0x00000400;	// enable rising edge (falling edge already on)
					ow_state = OW_ST_IDLE;
					opFlags = F_MEASURMENT_ALLOWED;
					break;						
				}
			}
			// expecting register address
			else if (owCmdState == OW_CMDST_FADDR)
			{					
				owRegAddr = owData;
				owCmdState = OW_CMDST_FLEN;
				ow_state = OW_ST_RD_TIMESLOT;
				break;
			}			
			else if (owCmdState == OW_CMDST_FLEN)
			{
				owDataLen = owData;
				ow_byteCount = 0;
				owCrc16 = 0x0000;

				if (owCmdCode == OW_CMD_READ_REG)
				{
					pOwTxData = &Dkst910RegMem[owRegAddr];
					ow_curByte = pOwTxData[ow_byteCount];
					owCrc16 = crc_16_update(owCrc16, ow_curByte);
					ow_state = OW_ST_WR_TIMESLOT;
				}
				else if (owCmdCode == OW_CMD_WRITE_REG)
				{					
					owCmdState = OW_CMDST_FDATA;
					ow_state = OW_ST_RD_TIMESLOT;
				}			
			}
			// if expecting a function data (follows the function command)
			else if (owCmdState == OW_CMDST_FDATA)
			{
				owDataBuf[ow_byteCount++] = owData;

				owCrc16 = crc_16_update(owCrc16, owData);

				// when full data stored, 
				if (ow_byteCount == owDataLen)
				{
					owCmdState = OW_CMDST_FCRC16;
					ow_byteCount = 0;
				}

				ow_state = OW_ST_RD_TIMESLOT;
				break;						
			}			
			// if expecting CRC16
			else if (owCmdState == OW_CMDST_FCRC16)
			{				
				ow_byteCount++;
				if (ow_byteCount == 2)
				{
					owRxCrc16 |= (uint16_t)owData;
					if (owRxCrc16 == owCrc16)
					{	
						ow_byteCount = 0;
						while ((owDataLen--) && (owRegAddr < DKST910_FIRST_RO_ADDR))
						{
							if (Dkst910RegMem[owRegAddr] !=  owDataBuf[ow_byteCount])
							{
								Dkst910RegMem[owRegAddr] = owDataBuf[ow_byteCount];
								opFlags |= F_REGWRITE_REQUIRED;
							}
							owRegAddr++;
							ow_byteCount++;
						}
												
						ow_curByte = OW_CMDACK;
					}
					else
					{
						ow_curByte = OW_CMDNACK;
					}

					ow_byteCount = 0; 
					owDataLen = 1;
					ow_writeEndState = OW_ST_WR_END;
					ow_readSampleState = OW_ST_RD_SAMPLE;
					ow_state = OW_ST_WR_TIMESLOT;
					break;
				}
				else
				{
					owRxCrc16 = (uint16_t)((owData << 8) & 0xff00);
				}

				ow_state = OW_ST_RD_TIMESLOT;
				break;
			}	
			else if (owCmdState == OW_CMDST_FMAGIC)
			{
				owDataBuf[ow_byteCount++] = owData;
				if (ow_byteCount == 2)
				{
					if ((*(uint16_t *)&owDataBuf[0]) == 0x5253)
					{
						ow_curByte = OW_CMDACK;
						opFlags |= F_SYSRESET_REQUESTED;
					}
					else
					{
						ow_curByte = OW_CMDNACK;
					}

					ow_byteCount = 0; 
					owDataLen = 1;
					ow_writeEndState = OW_ST_WR_END;
					ow_readSampleState = OW_ST_RD_SAMPLE;
					ow_state = OW_ST_WR_TIMESLOT;
					break;	
				}
				
				ow_state = OW_ST_RD_TIMESLOT;
				break;
			}
			// if expecting a ROM command
			else if (owCmdState == OW_CMDST_ROM)
			{					
				switch (owData)
				{
				// Match ROM command
				case OW_CMD_MATCHROM:
					ow_byteCount = 0;
					ow_curByte = ow_romCode[ow_byteCount];
					ow_readSampleState = OW_ST_RD_SAMPLE_MATCHROM;
					ow_state = OW_ST_RD_TIMESLOT;
					break;

				// Search ROM command received
				case OW_CMD_SEARCHROM:
					ow_SearchRomCode = ow_InitRomCode;
					ow_curByte = (unsigned char)(ow_SearchRomCode & 0x01);						
					ow_writeEndState = OW_ST_WR_END_SEARCHROM_1;
					ow_readSampleState = OW_ST_RD_SAMPLE_SEARCROM;
					ow_state = OW_ST_WR_TIMESLOT;
					break;
					
				// Skip ROM command, master told us to move to functional state
				// setup to read function command
				case OW_CMD_SKIPROM:
					owCmdState = OW_CMDST_FUNC;
					ow_state = OW_ST_RD_TIMESLOT;
					break;

				// Read ROM command, setup to send ROM code
				case OW_CMD_READROM:
					ow_byteCount = 0;
					ow_curByte = ow_romCode[ow_byteCount];
					ow_state = OW_ST_WR_TIMESLOT;
					break;
					
				// unsupported ROM command, back to idle state
				default:
					EXTI->RTSR |= 0x00000400;	// enable rising edge (falling edge already on)
					ow_state = OW_ST_IDLE;

					RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
					RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
					TIM3->PSC = 7;
					TIM3->CR1 |= TIM_CR1_CEN;
					
					opFlags = F_MEASURMENT_ALLOWED;
					break;
				}
			}		
		}		
		break;

	case OW_ST_RD_SAMPLE_MATCHROM:
		// sample the line and store next bit
		if (GPIOA->IDR & 0x0400)
			owData = 0x01;
		else
			owData = 0x00;

		// stop timer counter
		TIM3->CR1 &= ~TIM_CR1_CEN;

		if ((ow_curByte & 0x01) != owData)
		{
			EXTI->RTSR |= 0x00000400;	// enable rising edge (falling edge already on)
			ow_state = OW_ST_IDLE;

			RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
			TIM3->PSC = 7;
			TIM3->CR1 |= TIM_CR1_CEN;
			
			opFlags = F_MEASURMENT_ALLOWED;
			break;
		}

		ow_curByte >>= 1; 

		// if not received full 8 bits, move on to recieve next bit
		ow_bitCount++;
		if (ow_bitCount < 8)
		{
			ow_state = OW_ST_RD_TIMESLOT;
			break;
		}

		// received full 8 bits
		ow_bitCount = 0;
		ow_byteCount++;
		if (ow_byteCount == 8)
		{
			// full address matched, move to functional state
			ow_readSampleState = OW_ST_RD_SAMPLE;
			ow_state = OW_ST_RD_TIMESLOT;
			owCmdState = OW_CMDST_FUNC;
			break;
		}
		
		ow_curByte = ow_romCode[ow_byteCount];
		ow_state = OW_ST_RD_TIMESLOT;		
		break;

	case OW_ST_RD_SAMPLE_SEARCROM:
		// sample the line and store next bit
		if (GPIOA->IDR & 0x0400)
		{
			 if (ow_curByte & 0x01)
				goto stopRomSearch;			 
		}
		else
		{
			 if (!(ow_curByte & 0x01))
				goto stopRomSearch;
		}

		// stop timer counter
		TIM3->CR1 &= ~TIM_CR1_CEN;

		ow_SearchRomCode >>= 1;
		ow_curByte = (unsigned char)(ow_SearchRomCode & 0x01); 
		
		// if transmitted less than 8 bits, move to transmitting next bit
		ow_bitCount++;
		if (ow_bitCount < 64)
		{
			ow_writeEndState = OW_ST_WR_END_SEARCHROM_1;	
			ow_state = OW_ST_WR_TIMESLOT;
			break;
		}
		
stopRomSearch:		
		EXTI->RTSR |= 0x00000400;	// enable rising edge (falling edge already on)
		ow_state = OW_ST_IDLE;

		RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
		TIM3->PSC = 7;
		TIM3->CR1 |= TIM_CR1_CEN;
		
		opFlags = F_MEASURMENT_ALLOWED;
		break;

	case OW_ST_WR_END_SEARCHROM_1:
		// if current bit sent was '0'
		if (!(ow_curByte & 0x01))
		{
			// Release line 
			GPIOA->MODER &= ~0x00140000;	// configure PA9,10 as input
			ow_curByte = 0x01;
		}
		else
		{
			ow_curByte = 0x00;
		}

		// stop timer counter
		TIM3->CR1 &= ~TIM_CR1_CEN;

		ow_writeEndState = OW_ST_WR_END_SEARCHROM_2;
		ow_state = OW_ST_WR_TIMESLOT;
		break;

	case OW_ST_WR_END_SEARCHROM_2:
		// if current bit sent was '0'
		if (!(ow_curByte & 0x01))
		{
			// Release line 
			GPIOA->MODER &= ~0x00140000;	// configure PA9,10 as input				
		}

		// stop timer counter
		TIM3->CR1 &= ~TIM_CR1_CEN;

		ow_state = OW_ST_RD_TIMESLOT;
		break;
		
	case OW_ST_PRES_START:
		// disable GPIO IRQ (to avoid falling edge trigger from our own pulse)
		NVIC_DisableIRQ(EXTI4_15_IRQn);
		
		// generate presense pulse (pull data line low)
		GPIOA->BRR = 0x0600;			// clear port data output bits for PA9,10 to 0
		GPIOA->MODER |= 0x00140000;		// configure PA9,10 as output		

		ow_state = OW_ST_PRES_HOLD;

		TIM3->CNT = 0;
		TIM3->ARR = 80;				// setup timer to fire after 80 us
		break;

	case OW_ST_PRES_HOLD:
		TIM3->CR1 &= ~TIM_CR1_CEN;	// stop timer counter

		ow_bitCount = 0;
		ow_writeEndState = OW_ST_WR_END;
		ow_readSampleState = OW_ST_RD_SAMPLE;
		owCmdState = OW_CMDST_ROM;

		ow_state = OW_ST_PRES_HOLD_END;

		//EXTI->RTSR &= ~0x00000400;	// disable rising edge
		NVIC_EnableIRQ(EXTI4_15_IRQn);

		// Release data line (completes presence pulse)
		GPIOA->MODER &= ~0x00140000;	// configure PA9,10 as input

		break;	
	}
}

// 1-wire data line GPIO interrupt
// Triggers on falling edge and rising edge, depending on protocol state
void EXTI4_15_IRQHandler(void)
{
	uint16_t ow_curr_usec, ow_diff_usec;
	
	if( (EXTI->IMR & EXTI_IMR_MR10) && (EXTI->PR & EXTI_PR_PR10))
   	{
		EXTI->PR |= EXTI_PR_PR10;	

		switch (ow_state)
		{
		// Both falling and rising edge trigger in on
		case OW_ST_IDLE:
			ow_start_usec = TIM3->CNT;
			// If OW line low, this could be start of reset
			if (!(GPIOA->IDR & 0x0400))			
				ow_state = OW_ST_RESET;					
			
			break;

		case OW_ST_RESET:
			ow_curr_usec = TIM3->CNT;
			// data line high, possible reset completed
			if (GPIOA->IDR & 0x0400)
			{				
				// reset detected ?
				ow_diff_usec = (ow_curr_usec - ow_start_usec); 
				if ((ow_diff_usec >= 480) && (ow_diff_usec < 960))
				{					
					ow_start_usec = ow_curr_usec;
					ow_state = OW_ST_PRES_START;

					// stop measurement in progress
					opFlags = 0x00;
					
					TIM3->CR1 &= ~TIM_CR1_CEN;	// stop timer counter				
					TIM3->ARR = 30;				// set timer to generate event after 30 us
					TIM3->DIER |= 0x0001;		// enable update interrupt
					TIM3->CR1 |= TIM_CR1_CEN;	// re-enable timer counter
					NVIC_EnableIRQ(TIM3_IRQn);	// enable timer interrupt
				}
				else
				{
					ow_state = OW_ST_IDLE;
					opFlags = F_MEASURMENT_ALLOWED;
				}
			}
			else
			{
				ow_state = OW_ST_IDLE;
				opFlags = F_MEASURMENT_ALLOWED;
			}
			break;

		// falling edge only was enabled in this state
		// So start of read timeslot detected
		case OW_ST_RD_TIMESLOT:
			TIM3->CNT = 0;
			TIM3->ARR = 30;				// setup timer to fire after 30 us
			TIM3->CR1 |= TIM_CR1_CEN;	// re-enable timer counter
			ow_state = ow_readSampleState;
			break;

		// falling edge only was enabled in this state
		// So start of write timeslot detected
		case OW_ST_WR_TIMESLOT:
			TIM3->CNT = 0;
			TIM3->ARR = 35;				// setup timer to fire after 30 us
			TIM3->CR1 |= TIM_CR1_CEN;	// re-enable timer counter

			// to send '0' force line low for the rest of timeslot
			if (!(ow_curByte & 0x01))
			{				
				// pull the line low
				GPIOA->BRR = 0x0600;			// clear port data output bits for PA9,10 to 0
				GPIOA->MODER |= 0x00140000;		// configure PA9,10 as output
			}

			// to send '1' allow the line to return high (by not doing anything)
			ow_state = ow_writeEndState;
			break;

		case OW_ST_PRES_HOLD_END:			
			EXTI->RTSR &= ~0x00000400;	// disable rising edge			
			ow_state = OW_ST_RD_TIMESLOT;
			break;
		}		
	}	
}

// Configure ADC
static void adcConfigure(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// calibrate ADC
  	/* (1) Ensure that ADEN = 0 */
  	/* (2) Clear ADEN */ 
  	/* (3) Launch the calibration by setting ADCAL */
  	/* (4) Wait until ADCAL=0 */
  	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
    	ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  /* (2) */  

  	ADC1->CR |= ADC_CR_ADCAL; /* (3) */
  	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (4) */
		;
	
  	__NOP();__NOP();   /* This 2 NOPs are to ensure 2 ADC Cycles 
                        before setting ADEN bit  */	

	// Enable ADC
  	ADC1->CR |= ADC_CR_ADEN; /* (1) */
  	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */
		;

	// Select input channel ADC_IN0 
	ADC1->CHSELR |= 0x00000001;

	delayMicroseconds(60000);
	delayMicroseconds(60000);
	delayMicroseconds(60000);
}

// Periodic ADC processing
// Convert a value and return it
static unsigned short adcConvert(void)
{
	unsigned short val;
	
	ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */

	/* wait end of conversion */
	while (((ADC1->ISR & ADC_ISR_EOC) == 0) && (opFlags & F_MEASURMENT_ALLOWED))
		;

	val = ADC1->DR;

	return val;
}

/**
 * @brief   Integer square root for RMS
 * @param   sqrtAvg  (sum(x squared) /  count)
 * @retval                approximate square root
 *
 * Approximate integer square root, used for RMS calculations.
 */
static unsigned int sqrtI( unsigned long sqrtArg )
{
    unsigned int answer, x;
    unsigned long temp;
  
    if ( sqrtArg == 0 ) return 0;       // undefined result
    if ( sqrtArg == 1 ) return 1;       // identity
    answer = 0;             	// integer square root
    for( x=0x8000; x>0; x=x>>1 )
    {               // 16 bit shift
        answer |= x;            // possible bit in root
        temp = (unsigned long)(answer * answer); // fast unsigned multiply
        if (temp == sqrtArg) 
			break;    // exact, found it
			
        if (temp > sqrtArg) 
			answer ^= x;    // too large, reverse bit
    }
    return answer;          // approximate root
}

// Frequency detection. called for every sample
// Fx = Fo * (M/N)
// Fx - input signal frequency
// Fo - reference signal frequency (50 Hz)
// N - number of expected reference clock pulses (400) per measurement interval (400 periods)
// M - number of measured input signal pulses per measurement interval (400 periods)
static void detectFreq(uint16_t fCurrSample)
{
	static uint16_t fSampleCount = 0;
	static uint16_t fPrevSample, fPositivePulseCount;
	static uint8_t fLocked = 0;
	static uint8_t fWaitZeroCrossPositive = 0;
	static uint8_t fSineDirection;
	float freq;

	// Waiting zero cross event
	if (!fLocked)
	{
		if ((fSampleCount > 0) && (fCurrSample != fPrevSample))
		{
			// wait for a negative sample, when there is one we know our expected 'zero crossing' is rising front
			if (!fWaitZeroCrossPositive && (fCurrSample < V_DIGITAL_DEAD_MIN))
				fWaitZeroCrossPositive = 1;
					
			// Crossed zero ? then start measurement run
			if (fWaitZeroCrossPositive && (fCurrSample >= V_DIGITAL_DEAD_MAX)) 
			{
				fLocked = 1;
				fSampleCount = 0;
				fPositivePulseCount = 0;
				fSineDirection = DIRECTION_UP;
			}
		}
	}

	if ((fLocked) && (fCurrSample != fPrevSample) &&
		((fCurrSample >= V_DIGITAL_DEAD_MAX) || (fCurrSample < V_DIGITAL_DEAD_MIN)))
	{
		if (fSineDirection == DIRECTION_UP)
		{						
			if (fCurrSample < fPrevSample)
			{
				fSineDirection = DIRECTION_DOWN;

				if (fCurrSample > V_DIGITAL_MIDPOINT)
					fPositivePulseCount++;
			}
		}
		else
		{
			if (fCurrSample > fPrevSample)
			{
				fSineDirection = DIRECTION_UP;

				if (fCurrSample > V_DIGITAL_MIDPOINT)
					fPositivePulseCount--;
			}
		}		
	}

	fPrevSample = fCurrSample;
	fSampleCount++;
	if (fSampleCount == NUM_FREQ_SAMPLES)
	{
		if (!fLocked)
			freq = 0.0f;
		else 
			freq = (float)(RECCLK_FREQ * fPositivePulseCount / NUM_REFCLK_PULSES);

		*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_FREQ] = (uint16_t)((freq * 100.0f) + 0.5f);

		Dkst910FullStats[DKST910_FULL_STATS_ADDR_FREQ] = Dkst910RegMem[DKST910_REG_ADR_FREQ];
		Dkst910FullStats[DKST910_FULL_STATS_ADDR_FREQ+1] = Dkst910RegMem[DKST910_REG_ADR_FREQ+1];

		Dkst910ShortStats[DKST910_SHORT_STATS_ADDR_FREQ] = Dkst910RegMem[DKST910_REG_ADR_FREQ];
		Dkst910ShortStats[DKST910_SHORT_STATS_ADDR_FREQ+1] = Dkst910RegMem[DKST910_REG_ADR_FREQ+1];

		fLocked = 0;
		fWaitZeroCrossPositive = 0;
		fSampleCount = 0;
	}
}

// Monitor for under and over voltage and increment correct counters
// u16currRMSV current RMS Voltage of last half sine period
// profileNo - profile number 0=Prof 1; 1=Prof 2
static void monitorUnderOverVoltage(uint16_t u16currRMSV, int profileNo)
{
	dkst910_thres_state_t *pProfState = (dkst910_thres_state_t *)&thresState[profileNo];
	dkst910_prof_cfg_t *pProfCfg = (dkst910_prof_cfg_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF1_UVTRES + (profileNo * DKST910_PROF_SZ)];
	uint16_t condCurrTick, condDiffTick;
	uint8_t cntAddr, regId, fsId;
	
	if (pProfState->currCond == CONDITION_NORMAL)
	{
		if (u16currRMSV > pProfCfg->ovtres)
		{
			pProfState->condStartUs = TIM1->CNT;
			pProfState->condDurationMs = 0;
			pProfState->currCond = CONDITION_OVER_VOLTAGE;
		}
		else if (u16currRMSV < pProfCfg->uvtres)
		{
			pProfState->condStartUs = TIM1->CNT;
			pProfState->condDurationMs = 0;
			pProfState->currCond = CONDITION_UNDER_VOLTAGE;
		}
	}							
	else
	{
		condCurrTick = TIM1->CNT;
		condDiffTick = (condCurrTick - pProfState->condStartUs);
		if (condDiffTick >= 1000)
		pProfState->condDurationMs++;

		// if Voltage came back to within normal range
		if ((u16currRMSV <= pProfCfg->ovtres) && (u16currRMSV >= pProfCfg->uvtres))
		{
			// condition ended at duration thresState->condDurationMs
			// if event duration falls in the min max range of this profile, increment the counter corresponding to profileNo
			if ((pProfState->condDurationMs >= pProfCfg->min) && (pProfState->condDurationMs <= pProfCfg->max))
			{
				if (pProfState->currCond == CONDITION_OVER_VOLTAGE)
				{
					if (profileNo == 0)
					{
						cntAddr = DKST910_REG_ADR_CNT1_OV;
						regId = DKST910_FLASH_CNT1_OV;
						fsId = DKST910_FULL_STATS_ADDR_CNT1OV;
					}
					else
					{
						cntAddr = DKST910_REG_ADR_CNT2_OV;
						regId = DKST910_FLASH_CNT2_OV;
						fsId = DKST910_FULL_STATS_ADDR_CNT2OV;
					}					
				}
				else
				{
					if (profileNo == 0)
					{
						cntAddr = DKST910_REG_ADR_CNT1_UV;
						regId = DKST910_FLASH_CNT1_UV;
						fsId = DKST910_FULL_STATS_ADDR_CNT1UV;
					}
					else
					{
						cntAddr = DKST910_REG_ADR_CNT2_UV;
						regId = DKST910_FLASH_CNT2_UV;
						fsId = DKST910_FULL_STATS_ADDR_CNT2UV;
					}
				}

				// increment counter
				(*(uint32_t *)&Dkst910RegMem[cntAddr])++;
				
				Dkst910FullStats[fsId] = Dkst910RegMem[cntAddr];
				Dkst910FullStats[fsId+1] = Dkst910RegMem[cntAddr+1];
				Dkst910FullStats[fsId+2] = Dkst910RegMem[cntAddr+2];
				Dkst910FullStats[fsId+3] = Dkst910RegMem[cntAddr+3];
				
				FlashCounterRegisterNotifyChange(regId, *(uint32_t *)&Dkst910RegMem[cntAddr]);
			}

			pProfState->currCond = CONDITION_NORMAL;
		}
	}
}

// Set default configuation settings
static void set_default_config(void)
{
	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF1_UVTRES] = DKST910_DEF_PROF1_UNDERVOLTAGE;
	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF1_OVTRES] = DKST910_DEF_PROF1_OVERVOLTAGE;
	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF1_MIN] = DKST910_DEF_PROF1_MIN;
	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF1_MAX] = DKST910_DEF_PROF1_MAX;

	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF2_UVTRES] = DKST910_DEF_PROF2_UNDERVOLTAGE;
	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF2_OVTRES] = DKST910_DEF_PROF2_OVERVOLTAGE;
	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF2_MIN] = DKST910_DEF_PROF2_MIN;
	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_PROF2_MAX] = DKST910_DEF_PROF2_MAX;

	*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_BLKOUT_TRES] = DKST910_DEF_BLKOUT_TRES;
}

int main(void)
{	
	unsigned char *p;
	unsigned short currV, prevV, currVPlus, modV;
	unsigned int currRMSV;
	float fCurrRMSV;
	uint16_t u16currV;

	uint32_t curRMS, curHalfPdSumV, currVSq;
	uint16_t startTick, currTick, diffTick, isLockedOn;
	uint16_t zeroCrossDirection;

	memset((void *)Dkst910RegMem, 0, sizeof(Dkst910RegMem));

	// set version in the version register
	p = (unsigned char *)&Dkst910RegMem[DKST910_REG_ADR_VERSION];
	p[0] = DKSF_VER_MAJOR;
	p[1] = (unsigned char)(((DKSF_VER_MINOR << 4) & 0xf0) | (DKSF_VER_POINT));

	// load config registers from flash into RAM image of registers
	if (!FlashLoadCfgRegisters((uint16_t *)&Dkst910RegMem[0]))
	{
		// if flash contains no valid config, set "factory" defaults
		set_default_config();
		FlashWriteCfgRegisters((uint16_t *)&Dkst910RegMem[0]);
	}

	// Load flash registers
	FlashLoadCounterRegisters((uint32_t *)&Dkst910RegMem[DKST910_REG_ADR_CNT1_UV]);

	// pre-set full and short stats block length (1 byte)
	Dkst910FullStats[0] = (DKST910_FULL_STATS_BLOCK_SZ-1);
	Dkst910ShortStats[0] = (DKST910_SHORT_STATS_BLOCK_SZ-1);
	
	// copy version to full stats block
	memcpy((void *)&Dkst910FullStats[DKST910_FULL_STATS_ADDR_VER], 
		(void *)&Dkst910RegMem[DKST910_REG_ADR_VERSION], 2);

	// build version in the short stats block
	Dkst910ShortStats[1] = (unsigned char)(((DKSF_VER_MAJOR - 90) << 5) | (DKSF_VER_MINOR << 3) | DKSF_VER_POINT);

	// copy counters to full stats
	memcpy((void *)&Dkst910FullStats[DKST910_FULL_STATS_ADDR_CNT1UV],
		(void *)&Dkst910RegMem[DKST910_REG_ADR_CNT1_UV], (DKST910_NUM_COUNTERS * 4));

	get_device_id(&ow_romCode[1]);
	ow_romCode[7] = calc_crc(ow_romCode, 0x07);

	ow_InitRomCode = *(unsigned long long int *)&ow_romCode[0];
	
	// Enable peripheral clock for GPIO A
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 

	// configure PA4 as output (for LED use)
	gpio_configure(GPIOA, 4, GPIO_OTYPE_PUSHPULL, GPIO_MODE_OUT, GPIO_PUPDR_NONE);

	gpio_configure(GPIOA, 0, GPIO_OTYPE_OPENDRAIN, GPIO_MODE_ANALOG, GPIO_PUPDR_NONE);

	// Configure PA9 and PA10 as inputs - for 1Wire interface. It seems internal pull-up must be enabled
	gpio_configure(GPIOA, 9, GPIO_OTYPE_OPENDRAIN, GPIO_MODE_IN, GPIO_PUPDR_PULLUP);
	gpio_configure(GPIOA, 10, GPIO_OTYPE_OPENDRAIN, GPIO_MODE_IN, GPIO_PUPDR_PULLUP);

	// Enable peripheral clock for general purpose timer TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Set pre-scaler to make the timer freq=1 Mhz (tick every 1 us)
	// The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1)
	// system clock by default at 8 Mhz, so set prescaler 7
	TIM3->PSC = 7;
	// start timer
	TIM3->CR1 |= TIM_CR1_CEN;

	// Enable peripheral clock for advanced control timer TIM1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// Set pre-scaler to make the timer freq=1 Mhz (tick every 1 us)
	// The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1)
	// system clock by default at 8 Mhz, so set prescaler 7
	TIM1->PSC = 7;
	// start timer
	TIM1->CR1 |= TIM_CR1_CEN;

	// map PA10 to EXTI10 (SYSCFG_EXTICR3)
	SYSCFG->EXTICR[2] &= ~0xfffff0ff;
	// configure trigger on rising edge
	EXTI->RTSR |= 0x00000400;
	// configure trigger on falling edge
	EXTI->FTSR |= 0x00000400;
	// unmask irq line 10
	EXTI->IMR |= 0x00000400;
	// send interrupt priority
	NVIC_SetPriority (EXTI4_15_IRQn, 1); 

	// Configure ADC
	adcConfigure();
	
	// Enable interrupt
	NVIC_EnableIRQ(EXTI4_15_IRQn);

    while(1)
    {
    	if (!(opFlags & F_MEASURMENT_ALLOWED))
			continue;

		if (!(opFlags & F_MEASURMENT_RUNNING))
		{
			if (opFlags & F_SYSRESET_REQUESTED)
			{
				opFlags &= ~F_SYSRESET_REQUESTED;
				NVIC_SystemReset();
				while (1) ;
			}

			if (opFlags & F_REGWRITE_REQUIRED)
			{
				opFlags &= ~F_REGWRITE_REQUIRED;
				FlashWriteCfgRegisters((uint16_t *)&Dkst910RegMem[0]);				
			}
			
			opFlags |= F_MEASURMENT_RUNNING;
			sampleCount = 0;
			zeroCrossDirection = DIRECTION_NONE;
			isLockedOn = 0;
			curRMS = 0;
			startTick = TIM1->CNT;
		}
		
		currTick = TIM1->CNT;
		diffTick = (currTick - startTick);
		if (diffTick >= 1000)
		{
			startTick = currTick;
			
			// Time for next sample
			currV = adcConvert();

			if (!(opFlags & F_MEASURMENT_ALLOWED))
				continue;

			// frequency detection
			detectFreq(currV);

			// Waiting zero cross event
			if (!isLockedOn)
			{
				if ((sampleCount > 0) && (currV != prevV))
				{
					// wait for a negative sample, when there is one we know our expected 'zero crossing' is rising front
					if ((zeroCrossDirection == DIRECTION_NONE) && (currV < V_DIGITAL_MIDPOINT))
						zeroCrossDirection = DIRECTION_UP;
					
					// Crossed zero ?
					if ((zeroCrossDirection == DIRECTION_UP) && (currV >= V_DIGITAL_MIDPOINT)) 
					{
						isLockedOn = 1;
						curHalfPdSumV = 0;
						sampleCount = 0;
						halfPeriodSampleCount = 0;
					}
				}
			}

			if (isLockedOn)
			{
				if (sampleCount < NUM_ADC_SAMPLES)
				{
					adcSamples[sampleCount] = currV;

					// if sample is negative, convert to positive
					if (currV < V_DIGITAL_MIDPOINT)
						modV = V_DIGITAL_MIDPOINT + (V_DIGITAL_MIDPOINT - currV);
					else
						modV = currV;

					currVPlus = (modV - V_DIGITAL_MIDPOINT);

					currVSq = (uint32_t)(currVPlus * currVPlus);

					curRMS += currVSq;

					curHalfPdSumV += currVSq;

					// detect current sine half period
					halfPeriodSampleCount++;
					if (halfPeriodSampleCount == NUM_ADC_SAMPLES_HALF_PERIOD)
					{
						halfPeriodSampleCount = 0;
						// Calculate VRMS of the current half period
						curHalfPdSumV /= NUM_ADC_SAMPLES_HALF_PERIOD;
						currRMSV = sqrtI((unsigned long)curHalfPdSumV);
						fCurrRMSV = (float)(((currRMSV * 3.3f) / 4096.0f) * DIVISOR_220V);
						u16currV = (uint16_t)((fCurrRMSV * 1.0f) + 0.5f);

						monitorUnderOverVoltage(u16currV, 0);	// monitor profile 1
						monitorUnderOverVoltage(u16currV, 1);	// monitor profile 2
					}
				}
				else
				{
					// TODO: maybe we can store the results VRMSes of each half period
					// TODO: and sum them up and divide by 12 periods - to avoid double calculation?
					
					curRMS /= NUM_ADC_SAMPLES;
					currRMSV = sqrtI((unsigned long)curRMS);
					fCurrRMSV = (float)(((currRMSV * 3.3f) / 4096.0f) * DIVISOR_220V);
					currRMSV = (unsigned int)((fCurrRMSV * 10.0f) + 0.5f);

					*(uint16_t *)&Dkst910RegMem[DKST910_REG_ADR_VRMS] = (uint16_t)currRMSV;
					
					Dkst910FullStats[DKST910_FULL_STATS_ADDR_VRMS] = Dkst910RegMem[DKST910_REG_ADR_VRMS];
					Dkst910FullStats[DKST910_FULL_STATS_ADDR_VRMS+1] = Dkst910RegMem[DKST910_REG_ADR_VRMS+1];
					Dkst910ShortStats[DKST910_SHORT_STATS_ADDR_VRMS] = Dkst910RegMem[DKST910_REG_ADR_VRMS];
					Dkst910ShortStats[DKST910_SHORT_STATS_ADDR_VRMS+1] = Dkst910RegMem[DKST910_REG_ADR_VRMS+1];

					curRMS = 0;
					curHalfPdSumV = 0;
					sampleCount = 0;
					halfPeriodSampleCount = 0;
					prevV = currV;
					continue;
				}				
			}

			prevV = currV;
			sampleCount++;

			cntUpdateMs++;
			if (cntUpdateMs >= DKST910_FLASH_COUNTER_UPDATE_TIME)
			{
				cntUpdateMs = 0;
				FlashCounterRegistersUpdate();
			}
		}
    }
}
