#pragma once

typedef enum MCP23X17_REG_enum
{
	/*	IOCON BANK=0	*/
	IODIR = 0,		// I/O Direction, 1=input
	IPOL,			// Input polarity, 0=same
	GPINTEN,		// Interrupt-on-change enable, 0=disabled
	DEFVAL,			// Default pin value
	INTCON,			// Interrupt control, 0=previous state, 1=compare with DEFVAL
	IOCON,			// GPIO control
	GGPU,			// Input pull-up, 0=disabled
	INTF,			// Interrupt flags
	INTCAP,			// Port status at time of interrupt
	GPIO,			// Pin values
	OLAT,			// Output pin latches
} MCP23X17_REG_t;

#define PORTA_ADDR(reg, mode) reg*(2-mode)
#define PORTB_ADDR(reg, mode) reg*(2-mode)+9*mode+1

#define SEQ_ADDR 0
#define SEP_ADDR 1

#define MCP23X17_READ 1
#define MCP23X17_WRITE 0

/* IODIR */
#define MCP23X17_SET_PIN_AS_INPUT 1
#define MCP23X17_SET_PIN_AS_OUTPUT 0

/* IPOL */
#define MCP23X17_INVERT_INPUT 1
#define MCP23X17_FOLLOW_INPUT 0

/* GPINTEN */
#define MCP23X17_INT_ON_CHANGE_ENABLE 1
#define MCP23X17_INT_ON_CHANGE_DISABLE 0

/* INTCON */
#define MCP23X17_COMPARE_DEFAULT 1
#define MCP23X17_COMPARE_PREVIOUS 0

/* GPPU */
#define MCP23X17_INPUT_PULLUP_ENABLE 1
#define MCP23X17_INPUT_PULLUP_DISABLE 0

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