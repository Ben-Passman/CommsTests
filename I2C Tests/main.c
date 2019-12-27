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
#include "i2c_master.h"
#include "spi_master.h"

#define SLAVE1_ADDR 0x20
#define I2C_BUFF_LENGTH 16
#define SPI_BUFF_LENGTH 16

static uint8_t i2c_bytes[I2C_BUFF_LENGTH];
static uint8_t spi_tx_bytes[SPI_BUFF_LENGTH];
static uint8_t spi_rx_bytes[SPI_BUFF_LENGTH];

uint8_t system_init(void) 
{
	/*	GPIO	*/
	PORTC.PIN5CTRL = PORT_ISC_FALLING_gc; // Interrupt on button down
	PORTB.DIRSET = PIN2_bm | PIN3_bm;  // UART
	PORTA.DIRSET = PIN1_bm | PIN2_bm;	// TWI
	PORTA.PIN4CTRL = PORT_ISC_RISING_gc; // MCP PORTB interrupt
	
	/*	I/O Lines	*/
	PORTMUX.CTRLB = PORTMUX_TWI0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc | PORTMUX_USART0_DEFAULT_gc;
	
	/*	Interrupts	*/
	SREG |= CPU_I_bm;
	
	spi_master_init();
	i2c_master_init();
	
	spi_tx_bytes[2] = 0x67;

	return 0;
}

void mcp_write_callback(void)
{
	
}

void mcp_read_callback(void)
{
	spi_tx_bytes[0] = SLAVE1_ADDR<<1 | MCP23X17_WRITE;
	spi_tx_bytes[1] = PORTA_ADDR(OLAT, SEQ_ADDR);
	spi_tx_bytes[2] = (spi_rx_bytes[3] & 0x01)<<7 |		// PORTB PIN0 -> PORTA PIN7
					(spi_rx_bytes[3] & 0x04)<<4 |	// PORTB PIN2 -> PORTA PIN6
					(spi_rx_bytes[3] & 0x10)<<1 |	// PORTB PIN4 -> PORTA PIN5
					(spi_rx_bytes[3] & 0x40)>>2;	// PORTB PIN6 -> PORTA PIN4
	spi_start(spi_tx_bytes, spi_rx_bytes, 3, mcp_write_callback); // Write to MCP23X17, no callback required
}

void mcp_cycle_LEDS(void)
{
	spi_tx_bytes[2] = spi_tx_bytes[2] >> 7 | spi_tx_bytes[2] << 1;
	spi_tx_bytes[0] =  SLAVE1_ADDR<<1 | MCP23X17_WRITE;
	spi_tx_bytes[1] = PORTA_ADDR(OLAT, SEQ_ADDR);
	spi_start(spi_tx_bytes, spi_rx_bytes, 3, mcp_write_callback);

	i2c_bytes[0] = PORTA_ADDR(OLAT, SEQ_ADDR);
	i2c_bytes[1] = spi_tx_bytes[2];
	i2c_set_buffer(i2c_bytes, 2);
	i2c_start(SLAVE1_ADDR, I2C_WRITE_bm);
}

void mcp_read_inputs(void)
{
	spi_tx_bytes[0] = SLAVE1_ADDR<<1 | MCP23X17_READ;
	spi_tx_bytes[1] = PORTB_ADDR(INTCAP, SEQ_ADDR);
	spi_start(spi_tx_bytes, spi_rx_bytes, 3, mcp_read_callback);
}

int main(void)
{
	system_init();
	
	spi_start((const uint8_t *)0x1400, spi_rx_bytes, 16, mcp_read_callback); // Read MCP23X17 settings from EEPROM
	for (uint8_t i=0; i<15; i++)
	{
		i2c_bytes[i] = *(uint8_t *)(0x1401 + i);
	}
	i2c_set_buffer(i2c_bytes, 15);
	i2c_start(SLAVE1_ADDR, I2C_WRITE_bm);

	while (1)
    {
		
    }
}

ISR(PORTA_PORT_vect) // MCP PORTB interrupt, PINA4
{
	uint8_t intflags = PORTA.INTFLAGS;
	PORTA.INTFLAGS = intflags;
	mcp_read_inputs();
}

ISR(PORTC_PORT_vect) // Eval board button
{
	uint8_t intflags = PORTC.INTFLAGS;
	PORTC.INTFLAGS = intflags;
	mcp_cycle_LEDS();
}