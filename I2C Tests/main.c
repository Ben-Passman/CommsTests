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

#define I2C_ADDR_1 0x20
#define SPI_ADDR 0x20
#define I2C_BUFF_LENGTH 16
#define SPI_BUFF_LENGTH 4

static uint8_t LED_test = 0x67;
static uint8_t i2c_bytes[I2C_BUFF_LENGTH];
static uint8_t spi_bytes[SPI_BUFF_LENGTH];
//static uint8_t i2c_buff[I2C_BUFF_LENGTH];
//static uint8_t spi_buff[SPI_BUFF_LENGTH];

static uint8_t int_counter = 0;
static uint8_t read_counter = 0;

uint8_t system_init(void) 
{
	/*	GPIO	*/
	PORTC.PIN5CTRL = PORT_ISC_FALLING_gc; // Interrupt on button down
	PORTB.DIRSET = PIN2_bm | PIN3_bm;  // UART
	PORTA.DIRSET = PIN1_bm | PIN2_bm;	// TWI
	PORTA.PIN4CTRL = PORT_ISC_RISING_gc; // MCP PORTB interrupt (SPI)
	PORTA.PIN5CTRL = PORT_ISC_RISING_gc; // MCP PORTB interrupt (I2C)
	
	/*	I/O Lines	*/
	PORTMUX.CTRLB = PORTMUX_TWI0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc | PORTMUX_USART0_DEFAULT_gc;
	
	/*	Interrupts	*/
	SREG |= CPU_I_bm;
	
	spi_master_init();
	i2c_master_init();

	return 0;
}

void mcp_write_callback(void)
{
	
}

void mcp_read_callback(void)
{
	spi_bytes[0] = SPI_ADDR<<1 | MCP23X17_WRITE;
	spi_bytes[1] = PORTA_ADDR(OLAT, SEQ_ADDR);
	spi_bytes[2] = (spi_bytes[2] & 0x01)<<7 |		// PORTB PIN0 -> PORTA PIN7
					(spi_bytes[2] & 0x04)<<4 |	// PORTB PIN2 -> PORTA PIN6
					(spi_bytes[2] & 0x10)<<1 |	// PORTB PIN4 -> PORTA PIN5
					(spi_bytes[2] & 0x40)>>2;	// PORTB PIN6 -> PORTA PIN4
	spi_start(spi_bytes, spi_bytes, 3); // Write to MCP23X17, no callback required
	
	
}

void mcp_cycle_LEDS(void)
{
	LED_test = LED_test >> 7 | LED_test << 1;
	spi_bytes[0] =  SPI_ADDR<<1 | MCP23X17_WRITE;
	spi_bytes[1] = PORTA_ADDR(OLAT, SEQ_ADDR);
	spi_bytes[2] = LED_test;
	spi_start(spi_bytes, spi_bytes, 3);

	i2c_bytes[0] = PORTA_ADDR(OLAT, SEQ_ADDR);
	i2c_bytes[1] = LED_test;
	i2c_start(I2C_ADDR_1, I2C_WRITE_bm, i2c_bytes, 2);
}

void mcp_read_inputs(void)
{
	spi_bytes[0] = SPI_ADDR<<1 | MCP23X17_READ;
	spi_bytes[1] = PORTB_ADDR(INTCAP, SEQ_ADDR);
	// Counters confirm reads are being missed, possible data collision...
	// Need to check for busy status and queue messages.
	int_counter++;
	if (spi_start(spi_bytes, spi_bytes, 3) == SPI_BUSY)
	{
		read_counter++;
	}
	
// I2C read requires write to select register, then restart to transfer.
//	i2c_bytes[0] = PORTB_ADDR(INTCAP, SEQ_ADDR);
//	i2c_start(I2C_ADDR_1, I2C_WRITE_bm, i2c_bytes, 2); // Select register to read, then do restart
	i2c_start(I2C_ADDR_1, I2C_READ_bm, i2c_bytes, 16); // NEED TO CLEAN UP I2C READ FUNCTIONS
}

int main(void)
{
	uint8_t spi_rx_temp[16];
	
	system_init();
	spi_start((const uint8_t *)0x1400, spi_rx_temp, 16); // Read MCP23X17 settings from EEPROM	
	i2c_start(I2C_ADDR_1, I2C_WRITE_bm, (uint8_t *)0x1401, 15);

	while (1)
    {
		
    }
}

ISR(PORTA_PORT_vect) 
{
	uint8_t intflags = PORTA.INTFLAGS;
	PORTA.INTFLAGS = intflags;
	
	// MCP PORTB interrupt
	// PINA4 -> SPI
	// PINA5 -> I2C
	
	mcp_read_inputs();
}

ISR(PORTC_PORT_vect) // Eval board button
{
	uint8_t intflags = PORTC.INTFLAGS;
	PORTC.INTFLAGS = intflags;
	mcp_cycle_LEDS();
}