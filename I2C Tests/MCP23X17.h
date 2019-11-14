#pragma once

/* MCP23X17 Registers (IOCON BANK bit = 0) */
typedef enum MCP23X17_REG_enum
{
	/*	IOCON BANK=0	*/
	IODIRA0 = 0x00,		// I/O Direction, 1=input
	IPOLA0 = 0x02,		// Input polarity, 0=same
	GPINTENA0 = 0x04,	// Interrupt-on-change enable, 0=disabled
	DEFVALA0 = 0x06,	// Default pin value
	INTCONA0 = 0x08,	// Interrupt control, 0=previous state, 1=compare with DEFVAL
	IOCON0 = 0x0A,		// GPIO control
	GGPUA0 = 0x0C,		// Input pull-up, 0=disabled
	INTFA0 = 0x0E,		// Interrupt flags
	INTCAPA0 = 0x10,	// Port status at time of interrupt
	GPIOA0 = 0x12,		// Pin values
	OLATA0 = 0x14,		// Output pin latches
	IODIRB0 = 0x01,
	IPOLB0 = 0x03,
	GPINTENB0 = 0x05,
	DEFVALB0 = 0x07,
	INTCONB0 = 0x09,
	IOCONB0 = 0x0B,		// Equivalent to IOCON
	GGPUB0 = 0x0D,
	INTFB0 = 0x0F,
	INTCAPB0 = 0x11,
	GPIOB0 = 0x13,
	OLATB0 = 0x15,
	/*	IOCON BANK=1	*/
	IODIRA1 = 0x00,
	IPOLA1 = 0x01,
	GPINTENA1 = 0x02,
	DEFVALA1 = 0x03,
	INTCONA1 = 0x04,
	IOCON1 = 0x05,
	GGPUA1 = 0x06,
	INTFA1 = 0x07,
	INTCAPA1 = 0x08,
	GPIOA1 = 0x09,
	OLATA1 = 0x0A,
	IODIRB1 = 0x10,
	IPOLB1 = 0x11,
	GPINTENB1 = 0x12,
	DEFVALB1 = 0x13,
	INTCONB1 = 0x14,
	IOCONB1 = 0x15,
	GGPUB1 = 0x16,
	INTFB1 = 0x17,
	INTCAPB1 = 0x18,
	GPIOB1 = 0x19,
	OLATB1 = 0x1A,
} MCP23X17_REG_t;

/* IODIR */
#define MCP23X17_SET_PIN_AS_INPUT = 1;
#define MCP23X17_SET_PIN_AS_OUTPUT = 0;

/* IPOL */
#define MCP23X17_INVERT_INPUT = 1;
#define MCP23X17_FOLLOW_INPUT = 0;

/* GPINTEN */
#define MCP23X17_INT_ON_CHANGE_ENABLE = 1;
#define MCP23X17_INT_ON_CHANGE_DISABLE = 0;

/* INTCON */
#define MCP23X17_COMPARE_DEFAULT = 1;
#define MCP23X17_COMPARE_PREVIOUS = 0;

/* GPPU */
#define MCP23X17_INPUT_PULLUP_ENABLE = 1;
#define MCP23X17_INPUT_PULLUP_DISABLE = 0;

/* IOCON bit masks and bit positions */
#define IOCON_BANK_bm 0x80
#define IOCON_BANK_bp 7
#define IOCON_MIRROR_bm 0x40
#define IOCON_MIRROR_bp 6
#define IOCON_SEQOP_bm 0x20
#define IOCON_SEQOP_bp 5
#define IOCON_DISSLW_bm 0x10
#define IOCON_DISSLW_bp 4
#define IOCON_HAEN_bm 0x08
#define IOCON_HAEN_bp 3
#define IOCON_ODR_bm 0x04
#define IOCON_ODR_bp 2
#define IOCON_INTPOL_bm 0x02
#define IOCON_INTPOL_bp 1

#define IOCON_BANK_SEPARATE 0x80
#define IOCON_BANK_SEQUENTIAL 0x00
#define IOCON_INT_MIRRORED 0x40
#define IOCON_INT_SEPARATE 0x00
#define IOCON_ADDR_PTR_STATIC 0x20
#define IOCON_ADDR_PTR_SEQUENTIAL 0x00
#define IOCON_SLEW_RATE_DISABLED 0x10
#define IOCON_SLEW_RATE_ENABLED 0x00
#define IOCON_HW_ADDR_EN 0x08
#define IOCON_INT_OPEN_DRAIN 0x04
#define IOCON_INT_ACTIVE_DRIVER 0x00
#define IOCON_INT_ACTIVE_HIGH 0x02
#define IOCON_INT_ACTIVE_LOW 0x00