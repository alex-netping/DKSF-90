/*
 * DKST 910.5 220V 1-Wire power sensor
 * definitions
 */
#ifndef DKST910_H
#define DKST910_H

// To build the code for the "Waveform generator" boards uncomment #define DKST910_GENERATOR
// If #define DKST910_GENERATOR is commented out, the build is for Live 220V device
//#define DKST910_GENERATOR

#define DKSF_VER_MAJOR			90
#define DKSF_VER_MINOR			1
#define DKSF_VER_POINT			0

#define GPIO_MODE_MASK			0x03
#define GPIO_MODE_IN			0x00
#define GPIO_MODE_OUT			0x01
#define GPIO_MODE_AF			0x02
#define GPIO_MODE_ANALOG		0x03

#define GPIO_PUPDR_MASK			0x3
#define GPIO_PUPDR_NONE			0x0
#define GPIO_PUPDR_PULLUP		0x1
#define GPIO_PUPDR_PULLDOWN		0x2

#define GPIO_SPEED_DEFAULT		0xffffffff
#define GPIO_SPEED_HIGH			0x03
#define GPIO_SPEED_MASK			0x03

#define GPIO_OTYPE_PUSHPULL		0
#define GPIO_OTYPE_OPENDRAIN	1

#define GPIO_AF_MASK			0xf

#define BOARD_CLOCK_MHZ			8000000

#define OW_CMD_READROM			0x33	// 1-Wire ROM command 'Read ROM'
#define OW_CMD_SKIPROM			0xCC	// 1-Wire ROM command 'Skip ROM'
#define OW_CMD_SEARCHROM		0xF0	// 1-Wire ROM command 'Search ROM'
#define OW_CMD_MATCHROM			0x55	// 1-Wire ROM command 'Match ROM'

// This command consists of two parts: high 4 bits = Read cmd (6); low 4 bits=register number
// when low 4 bits equals '0' - read all registers
#define OW_CMD_READ_REG			0x60	// Read register(s)
#define OW_CMD_READ_STATS		0x62	// Read stats full version
#define OW_CMD_READ_SHORTSTATS		0x64	// Read stats short
#define OW_CMD_SYSRESET			0xA2	// Reset MCU

// This command consists of two parts: high 4 bits = Write cmd (4); low 4 bits=register number
// when low 4 bits equals '0' - write all registers
#define OW_CMD_WRITE_REG		0x40	// Write register(s)

#define OW_CMDACK				0x06	// Acknowledge
#define OW_CMDNACK				0x15	// Negative ack

// Definition of DKST 910.5 Sensor registers
#define DKST910_REG_ADR_PROF1_UVTRES		0x00	// Counter Profile 1: Under voltage threshold
#define DKST910_REG_ADR_PROF1_OVTRES		0x02	// Counter Profile 1: Over voltage threshold
#define DKST910_REG_ADR_PROF1_MIN 			0x04	// Counter Profile 1: Min duration
#define DKST910_REG_ADR_PROF1_MAX 			0x06	// Counter Profile 1: Max duration
#define DKST910_REG_ADR_PROF2_UVTRES		0x0c	// Counter Profile 2: Under voltage threshold
#define DKST910_REG_ADR_PROF2_OVTRES		0x0e	// Counter Profile 2: Over voltage threshold
#define DKST910_REG_ADR_PROF2_MIN 			0x10	// Counter Profile 2: Min duration
#define DKST910_REG_ADR_PROF2_MAX 			0x12	// Counter Profile 2: Max duration
#define DKST910_REG_ADR_BLKOUT_TRES			0x18	// Blackout threshold duration
#define DKST910_REG_ADR_RESERVED			0x1a
#define DKST910_REG_ADR_CNT1_UV				0x1c	// Counter 1 Under-Voltage
#define DKST910_REG_ADR_CNT1_OV				0x20	// Counter 1 Over-Voltage
#define DKST910_REG_ADR_CNT2_UV				0x24	// Counter 2 Under-Voltage
#define DKST910_REG_ADR_CNT2_OV				0x28	// Counter 2 Over-Voltage
#define DKST910_REG_ADR_CNT_BLKOUT			0x2c	// Blackout counter
#define DKST910_REG_ADR_VRMS				0x30	// V RMS
#define DKST910_REG_ADR_FREQ				0x32	// Frequency
#define DKST910_REG_ADR_VERSION				0x34	// Firmware version

// Last register address
#define DKST910_REG_ADR_LAST		(DKST910_REG_ADR_VERSION)

#define DKST910_REGMEM_SZ					54		// Byte size of register memory area

#define DKST910_PROF_SZ						12

#define DKST910_FULL_STATS_ADDR_VER			0x01
#define DKST910_FULL_STATS_ADDR_CNT1UV		0x03
#define DKST910_FULL_STATS_ADDR_CNT1OV		0x07
#define DKST910_FULL_STATS_ADDR_CNT2UV		0x0B
#define DKST910_FULL_STATS_ADDR_CNT2OV		0x0F
#define DKST910_FULL_STATS_ADDR_CNTBO		0x13

#define DKST910_FULL_STATS_ADDR_VRMS		0x17
#define DKST910_FULL_STATS_ADDR_FREQ		0x19

#define DKST910_SHORT_STATS_ADDR_VRMS		0x07
#define DKST910_SHORT_STATS_ADDR_FREQ		0x09

#define DKST910_FULL_STATS_BLOCK_SZ			27
#define DKST910_SHORT_STATS_BLOCK_SZ		11

// addres of first read only register
#define DKST910_FIRST_RO_ADDR				(DKST910_REG_ADR_CNT1_UV)

typedef enum
{
	OW_ST_IDLE,							// Wait falling edge - start of reset	
	OW_ST_RD_TIMESLOT,					// Wait falling edge - start of read timeslot
	OW_ST_RD_SAMPLE,					// Wait before sampling the line
	OW_ST_WR_TIMESLOT,					// Wait falling edge - start of write timeslot
	OW_ST_WR_END,						// Wait completion of write timelsot

	OW_ST_WR_END_STATS,
	OW_ST_WR_END_STATS2,
	OW_ST_WR_END_CRC,
	
	OW_ST_WR_END_SEARCHROM_1,
	OW_ST_WR_END_SEARCHROM_2,
	
	OW_ST_RD_SAMPLE_SEARCROM,
	OW_ST_RD_SAMPLE_MATCHROM,
} ow_state_t;

typedef enum
{
	OW_CMDST_ROM,						// receiving or replying to ROM command
	OW_CMDST_FUNC,						// receiving or replying to Function command
	OW_CMDST_FADDR,						// receiving register address
	OW_CMDST_FLEN,						// receiving data length
	OW_CMDST_FDATA,						// receiving function command data
	OW_CMDST_FCRC16,					// receiving function cmd data CRC16
	OW_CMDST_FMAGIC,					// receiving 16 bit magic (part of reset command)
} ow_cmd_state_t;

#define F_MEASURMENT_RUNNING		0x01
#define F_MEASURMENT_ALLOWED		0x02
#define F_REGWRITE_REQUIRED			0x04	// It is required to commit config registers to flash
#define F_SYSRESET_REQUESTED		0x80

// Measurement interval for VRMS = 12 sine periods = 20 ms * 12 = 240 ms
// 1 Sample per millisecond
#define NUM_ADC_SAMPLES	240
//#define NUM_ADC_SAMPLES	480

#define NUM_ADC_SAMPLES_HALF_PERIOD	10
#define NUM_ADC_SAMPLES_FULL_PERIOD	20

// Measure frequency over 400 periods
#define NUM_REFCLK_PULSES 400
//#define NUM_REFCLK_PULSES 12
//#define NUM_REFCLK_PULSES 24

#define NUM_FREQ_SAMPLES (NUM_REFCLK_PULSES * NUM_ADC_SAMPLES_FULL_PERIOD)

#define RECCLK_FREQ 50.0f

#define DIRECTION_NONE		0	// sine direction unknown
#define DIRECTION_UP		1	// sine direction is up
#define DIRECTION_DOWN		2	// sine direction is down

// Generator board digital levels
#ifdef DKST910_GENERATOR
#define V_DIGITAL_MIDPOINT		1231 // 0.991 V
#define V_DIGITAL_MIN			851 // 0.685 V
#define V_DIGITAL_DEAD_ZONE		95	// 25% above and below zero line
#define DIVISOR_220V 1100.0f
#else
// Real (220V) board digital levels
#define V_DIGITAL_MIDPOINT		1524 // 1.227 V
#define V_DIGITAL_MIN			1200 // 0.966 V
#define V_DIGITAL_DEAD_ZONE		81	// 25% above and below zero line
#define DIVISOR_220V 1320.0f
#endif

#define V_DIGITAL_DEAD_MAX		(V_DIGITAL_MIDPOINT + V_DIGITAL_DEAD_ZONE)	
#define V_DIGITAL_DEAD_MIN		(V_DIGITAL_MIDPOINT - V_DIGITAL_DEAD_ZONE)	

#define CONDITION_NORMAL			0
#define CONDITION_OVER_VOLTAGE		1
#define CONDITION_UNDER_VOLTAGE		2

typedef struct
{
	unsigned char currCond;
	uint16_t	condStartUs;
	uint16_t	condDurationMs;
} dkst910_thres_state_t;

typedef struct
{
	uint16_t	uvtres;
	uint16_t	ovtres;
	uint16_t	min;
	uint16_t	max;
	uint16_t	reserved1;
	uint16_t	reserved2;
} dkst910_prof_cfg_t;

#define DKST910_DEF_PROF1_UNDERVOLTAGE			160		// ~30% below nominal 230V 
#define DKST910_DEF_PROF1_OVERVOLTAGE			300		// ~30% above nominal 230V 
#define DKST910_DEF_PROF1_MIN					25		// duration
#define DKST910_DEF_PROF1_MAX					35		// duration

#define DKST910_DEF_PROF2_UNDERVOLTAGE			207		// 10% below nominal 230V 
#define DKST910_DEF_PROF2_OVERVOLTAGE			253		// 10% above nominal 230V
#define DKST910_DEF_PROF2_MIN					36
#define DKST910_DEF_PROF2_MAX					65000
#define DKST910_DEF_BLKOUT_TRES					240

#define DKST910_FLASH_COUNTER_UPDATE_TIME		30000	// number of milliseconds between flash writes (if any counters changed)

#endif // DKST910_H

