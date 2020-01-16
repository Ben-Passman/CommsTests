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
	- MCP23X17 require reset on output pins before use. Pins are input by default.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "MCP23X17.h"
#include "i2c_master.h"
#include "spi_master.h"
#include "msg_buffer.h"

#define I2C_ADDR_1 0x20
#define SPI_ADDR 0x20

#define BUFF_SIZE 4
#define I2C_BUFF_LENGTH 16
#define SPI_BUFF_LENGTH 4
#define SPI_SETUP (const uint8_t *)0x1400 // MCP23X17 settings from EEPROM
#define I2C_SETUP (uint8_t *)0x1401

static uint8_t LED_test = 0x67;
static uint8_t INTCAP_ADDR = PORTB_ADDR(INTCAP, SEQ_ADDR);
static uint8_t i2c_rx_buffer;
static uint8_t i2c_tx_buffer[I2C_BUFF_LENGTH] = {PORTA_ADDR(OLAT, SEQ_ADDR), 0};
static uint8_t spi_bytes[SPI_BUFF_LENGTH];

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

/*	****************	USART setup for debugging	****************	
// 20MHz clock, 6x prescaler -> 3.333MHz CLK_PER
//#define F_CPU (20E6/2)
//#define BAUD_RATE 57600
void usart_init()
{
	USART0.CTRLB = USART_TXEN_bm;
	USART0.BAUD = 231; //(3333333 * 64.0) / (BAUD_RATE * 16.0);
}
void usart_put_c(uint8_t c)
{
	VPORTB.DIR |= PIN2_bm | PIN6_bm;  //picoPower 2b: see Disable Tx below
	USART0.STATUS = USART_TXCIF_bm;
	
	VPORTB.OUT |= PIN6_bm;
	USART0.TXDATAL = c;
	while(!(USART0.STATUS & USART_TXCIF_bm));
	VPORTB.OUT &= ~PIN6_bm;
	VPORTB.DIR &= ~PIN2_bm | PIN6_bm;
	//picoPower 2b: Disable Tx pin in-between transmissions
}
	********************************		********************************	*/

void mcp_i2c_callback(void)
{
	// Called when TX/RX complete. Check data buffer and trigger restart if appropriate.
	if(Q_RW == 1)
	{
		i2c_tx_buffer[1] = i2c_rx_buffer;
		add_to_msg_queue(I2C_ADDR_1, I2C_WRITE_bm, i2c_tx_buffer, 2);
	}
	delete_from_msg_queue();
	if(get_msg_queue_status() != RB_EMPTY)
	{
		i2c_set_buffer(Q_ADDR<<1 | Q_RW, Q_DATA, Q_DATA_LEN);
		i2c_set_restart();
	}
	else
	{
		i2c_set_stop();
	}
}

void mcp_cycle_LEDS(void)
{
	LED_test = LED_test >> 7 | LED_test << 1;
	spi_bytes[0] =  SPI_ADDR<<1 | MCP23X17_WRITE;
	spi_bytes[1] = PORTA_ADDR(OLAT, SEQ_ADDR);
	spi_bytes[2] = LED_test;
	spi_start(spi_bytes, spi_bytes, 3);

	// Buffer may be overwritten this way if queue is large. Use separate Read/Write buffers.
	i2c_tx_buffer[1] = LED_test;
	add_to_msg_queue(I2C_ADDR_1, I2C_WRITE_bm, i2c_tx_buffer, 2);
	
	if(i2c_idle())
	{
		i2c_set_buffer(Q_ADDR<<1 | Q_RW, Q_DATA, Q_DATA_LEN);
		i2c_start(mcp_i2c_callback);	
	}
	
}

void mcp_read_inputs(void)
{
	spi_bytes[0] = SPI_ADDR<<1 | MCP23X17_READ;
	spi_bytes[1] = PORTB_ADDR(INTCAP, SEQ_ADDR);
	spi_start(spi_bytes, spi_bytes, 3);
	
// I2C read requires write to select register, then restart to transfer.
	add_to_msg_queue(I2C_ADDR_1, I2C_WRITE_bm, &INTCAP_ADDR, 1);
	add_to_msg_queue(I2C_ADDR_1, I2C_READ_bm, &i2c_rx_buffer, 1);
	
	if(i2c_idle())
	{
		i2c_set_buffer(Q_ADDR<<1 | Q_RW, Q_DATA, Q_DATA_LEN);
		i2c_start(mcp_i2c_callback);	
	}
}

int main(void)
{
	uint8_t spi_rx_temp[16]; // Used once, should free up this memory after use.
	
	system_init();
//	usart_init();
	clear_msg_queue();
	spi_start(SPI_SETUP, spi_rx_temp, 16); // Read MCP23X17 settings from EEPROM	
	i2c_set_buffer((I2C_ADDR_1<<1) | I2C_WRITE_bm, I2C_SETUP, 15);
	i2c_start(mcp_i2c_callback);

	while (1)
    {
		
    }
}

ISR(PORTA_PORT_vect) 
{
	uint8_t intflags = PORTA.INTFLAGS;
	PORTA.INTFLAGS = intflags;
	
	// MCP PORTB interrupt, PINA4 -> SPI, PINA5 -> I2C
	mcp_read_inputs();
}

ISR(PORTC_PORT_vect) // Eval board button
{
	uint8_t intflags = PORTC.INTFLAGS;
	PORTC.INTFLAGS = intflags;
	mcp_cycle_LEDS();
}