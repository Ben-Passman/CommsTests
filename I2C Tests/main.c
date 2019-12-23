/*
 * I2C Tests.c
 *
 * Created: 13/10/2019 10:34:51 AM
 * Author : Ben

	ATTiny817
	- 8kB Flash		0x8000	->	Program storage
	- 128B EEPROM	0x1400	->	Data Memory
	- 512B SRAM		0x3E00	->	Data storage & stack
	
	From 817 evaluation board schematics:
	- LED turned on by driving PC0 high.
	- User button is low-active on PC5, with external pullup.
	
	Notes:
	- MCP23X17 require reset on output pins before use. Default register values following a reset do not enable output pins.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "MCP23X17.h"
#include "spi_master.h"

#define SLAVE1_ADDR 0x20

static uint8_t tx_bytes[] = {	0x40, OLATA0, 0x67	}; // Need to initialise expander IO port before this will work (Set IODRA to 0x00).
static uint8_t rx_bytes[16];

uint8_t system_init() 
{
	/*	GPIO	*/
	PORTC.PIN5CTRL = PORT_ISC_FALLING_gc; // Interrupt on button down
	PORTB.DIRSET = PIN2_bm | PIN3_bm;  // UART
	PORTA.DIRSET = PIN1_bm | PIN2_bm;	// TWI
	
	/*	I/O Lines	*/
	PORTMUX.CTRLB = PORTMUX_TWI0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc | PORTMUX_USART0_DEFAULT_gc;
	
	/*	Interrupts	*/
	SREG |= CPU_I_bm;
	
	spi_master_init();
	
	return 0;
}

int main(void)
{
	system_init();
	
	spi_set_buffers((const uint8_t *)0x1400, rx_bytes, 16); // EEPROM read
	spi_start();
	
	while (1)
    {
		
    }
}

ISR(PORTC_PORT_vect)
{
	uint8_t intflags = PORTC.INTFLAGS;
	PORTC.INTFLAGS = intflags;
	
	spi_set_buffers(tx_bytes, rx_bytes, 3);
	spi_start();
	tx_bytes[2] = tx_bytes[2] >> 7 | tx_bytes[2] << 1;
}