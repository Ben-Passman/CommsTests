#include <avr/io.h>
#include <avr/interrupt.h>
#include "MCP23X17.h"
#include "i2c_master.h"
/*
 * I2C Tests.c
 *
 * Created: 13/10/2019 10:34:51 AM
 * Author : Ben

	ATTiny817
	- 8kB Flash		0x8000	->	Program storage
	- 128B EEPROM	0x1400	->	Data Memory
	- 512B SRAM		0x3E00	->	Data storage & stack
	
	From schematics:
	- LED turned on by driving PC0 high.
	- User button is low-active on PC5, with external pullup.
	
	Notes:
	- TWI0 uses MADDR and MDATA registers for data transfer. MADDR contains a 7-bit address and the R/W bit, MDATA contains 8 bits of data.
	- MCP23017 requires reset on output pins before use. Default register values following a reset do not enable output pins.
	- Test message works, need to decouple i2c code and add basic error handling.
*/

#define SLAVE1_ADDR 0x20

uint8_t bytes[] = {	OLATA0, 0x67	}; // Need to initialise expander IO port before this will work. IODRA, 0x00

uint8_t system_init() 
{
	/*	GPIO	*/
	PORTC.DIRSET = PIN0_bm; // Set onboard LED as output
	PORTC.PIN5CTRL = PORT_ISC_FALLING_gc; // Interrupt on button down
	PORTB.DIRSET = PIN2_bm | PIN3_bm;  // UART
	PORTA.DIRSET = PIN1_bm | PIN2_bm;	// TWI
	
	/*	I/O Lines	*/
	PORTMUX.CTRLB = PORTMUX_TWI0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc | PORTMUX_USART0_DEFAULT_gc;
	
	/*	Interrupts	*/
	SREG |= CPU_I_bm;
	
	i2c_master_init();
	
	return 0;
}

int main(void)
{
	system_init();
	
	// Should use (const unint8_t *) for EEPROM.
	i2c_set_buffer((uint8_t *)0x1400, 2);
	i2c_start(SLAVE1_ADDR, I2C_WRITE_bm);
	
	while (1)
    {
		
    }
}

ISR(PORTC_PORT_vect)
{
	uint8_t intflags = PORTC.INTFLAGS;
	PORTC.INTFLAGS = intflags;
	// Debug
	PORTC.OUTTGL = PIN0_bm;
	
	i2c_set_buffer(bytes, 2);
	i2c_start(SLAVE1_ADDR, I2C_WRITE_bm);
	bytes[1] = bytes[1] >> 7 | bytes[1] << 1;
}