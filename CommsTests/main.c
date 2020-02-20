/*
 * Comms Tests
 * main.c
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
#define I2C_BUFF_LENGTH 2
#define SPI_BUFF_LENGTH 4
#define SPI_SETUP (const uint8_t *)0x1400 // MCP23X17 settings from EEPROM
#define I2C_SETUP (uint8_t *)0x1401

static uint8_t LED_test = 0x67;
static uint8_t target_reg;
static uint8_t i2c_rx_buffer;
// static uint8_t spi_rx_buffer;
static uint8_t i2c_tx_buffer[I2C_BUFF_LENGTH];
static struct ring_buffer i2c_rb = {.head=0, .tail=0, .status=RB_EMPTY, .overflow=0};
static uint8_t spi_bytes[SPI_BUFF_LENGTH];

uint8_t system_init(void) 
{
	/*	GPIO	*/
	PORTC.PIN5CTRL = PORT_ISC_FALLING_gc; // Interrupt on button down
	PORTB.DIRSET = PIN2_bm | PIN3_bm;  // UART
	PORTA.DIRSET = PIN1_bm | PIN2_bm;	// TWI
	PORTA.PIN4CTRL = PORT_ISC_RISING_gc; // MCP PORTB interrupt (SPI)
	PORTA.PIN5CTRL = PORT_ISC_RISING_gc; // MCP PORTB interrupt (I2C)
	// PA6 and PA7 for QTouch Buttons
	
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

/*	********************************************************************************
	I2C Callback functions
	********************************************************************************	*/

i2c_operations_t i2c_rx_cb (void)
{
	i2c_operations_t next_op = stop_i2c;
	delete_from_msg_queue(&i2c_rb);

/*i2c_tx_buffer[0] = PORTA_ADDR(OLAT, SEQ_MODE);
i2c_tx_buffer[1] = (i2c_rx_buffer&0x01)<<7 | (i2c_rx_buffer&0x04)<<4 | (i2c_rx_buffer&0x10)<<1 | (i2c_rx_buffer&0x40)>>2;
add_to_msg_queue(&i2c_rb, I2C_ADDR_1, I2C_WRITE_bm, i2c_tx_buffer, 2);*/
	
	if(i2c_rb.status != RB_EMPTY)
	{
		i2c_set_buffer(Q_ADDR(i2c_rb)<<1 | Q_RW(i2c_rb), Q_DATA(i2c_rb), Q_DATA_LEN(i2c_rb));
		next_op = restart_i2c;
	}
	
	return next_op;
}

i2c_operations_t i2c_tx_cb (void)
{
	i2c_operations_t next_op = stop_i2c;
	delete_from_msg_queue(&i2c_rb);

	if(i2c_rb.status != RB_EMPTY)
	{
		i2c_set_buffer(Q_ADDR(i2c_rb)<<1 | Q_RW(i2c_rb), Q_DATA(i2c_rb), Q_DATA_LEN(i2c_rb));
		next_op = restart_i2c;
	}
	
	return next_op;
}

/*	************************************************************************************************

	************************************************************************************************	*/

void mcp_cycle_LEDS(void)
{
	LED_test = LED_test >> 7 | LED_test << 1;
	spi_bytes[0] =  SPI_ADDR<<1 | MCP23X17_WRITE;
	spi_bytes[1] = PORTA_ADDR(OLAT, SEQ_MODE);
	spi_bytes[2] = LED_test;
	spi_start(spi_bytes, spi_bytes, 3);

	i2c_tx_buffer[0] = PORTA_ADDR(OLAT, SEQ_MODE);
	i2c_tx_buffer[1] = LED_test;
	add_to_msg_queue(&i2c_rb, I2C_ADDR_1, I2C_WRITE_bm, i2c_tx_buffer, 2);
	
	if(i2c_idle())
	{
		i2c_set_buffer(Q_ADDR(i2c_rb)<<1 | Q_RW(i2c_rb), Q_DATA(i2c_rb), Q_DATA_LEN(i2c_rb));
		i2c_start();	
	}
	
}

void mcp_read_inputs(void)
{
	spi_bytes[0] = SPI_ADDR<<1 | MCP23X17_READ;
	spi_bytes[1] = PORTB_ADDR(INTCAP, SEQ_MODE);
	spi_start(spi_bytes, spi_bytes, 3);
	
	target_reg = PORTB_ADDR(GPIO, SEQ_MODE); // INTCAP, SEQ_MODE);
	add_to_msg_queue(&i2c_rb, I2C_ADDR_1, I2C_WRITE_bm, &target_reg, 1);	// Select register
	add_to_msg_queue(&i2c_rb, I2C_ADDR_1, I2C_READ_bm, &i2c_rx_buffer, 1);	// Read
	
	if(i2c_idle())
	{
		i2c_set_buffer(Q_ADDR(i2c_rb)<<I2C_READ_bm | Q_RW(i2c_rb), Q_DATA(i2c_rb), Q_DATA_LEN(i2c_rb));
		i2c_start();	
	}
}

ISR(PORTA_PORT_vect) // MCP PORTB interrupts, PINA4 -> SPI, PINA5 -> I2C
{
	uint8_t intflags = PORTA.INTFLAGS;
	PORTA.INTFLAGS = intflags;
	//mcp_read_inputs();
	mcp_cycle_LEDS();
}

ISR(PORTC_PORT_vect) // Eval board button
{
	uint8_t intflags = PORTC.INTFLAGS;
	PORTC.INTFLAGS = intflags;
	//mcp_cycle_LEDS();
	mcp_read_inputs();
}

int main(void)
{
	uint8_t spi_rx_temp[16]; // Used once, should free up this memory after use.
	
	system_init();
//	usart_init();
	clear_msg_queue(&i2c_rb);
	i2c_set_event_callback(tx_complete, i2c_tx_cb);
	i2c_set_event_callback(rx_complete, i2c_rx_cb);
	spi_start(SPI_SETUP, spi_rx_temp, 16); // Read MCP23X17 settings from EEPROM	
	i2c_set_buffer((I2C_ADDR_1<<1) | I2C_WRITE_bm, I2C_SETUP, 15);
	i2c_start();

	while (1)
    {

    }
}