#include "spi_master.h"
#include <avr/interrupt.h>

typedef struct
{
	uint8_t slave_addr;
	uint8_t *byte_array;
	uint8_t size_byte_array;
	uint8_t byte_count;
} spi_data;

spi_data spi_io = {0};

void spi_slave_select(void)
{
	VPORTC.OUT &= ~PIN3_bm; // Set SS low
}

void spi_slave_deselect(void)
{
	VPORTC.OUT |= PIN3_bm; // Set SS high
}

/*	------------------------------------------------------------------------------------------------
	INTERRUPTS
	------------------------------------------------------------------------------------------------	*/

void spi_isr (void)
{
	/*
		For non-buffer mode:
		SPI_IF_bm is set on transfer complete. Cleared on interrupt vector execution, or by SPI.INTFLAGS read followed by DATA access.
		SPI_WRCOL is set if DATA is written before shift out is complete.
	*/
	
	if (spi_io.byte_count < spi_io.size_byte_array)
	{
		SPI0.DATA = *(spi_io.byte_array + spi_io.byte_count);
		spi_io.byte_count ++;
	}
	else
	{
		spi_slave_deselect();
	}
}

ISR(SPI0_INT_vect)
{
	register8_t int_flags = SPI0.INTFLAGS;
	// SPI_WRCOL_bm		write collision
	// SPI_IF_bm		byte transfer complete
	
	if (int_flags & SPI_WRCOL_bm)
	{
		spi_slave_deselect();
	} else
	{
		spi_isr();
	}
	SPI0.INTFLAGS = 0x00;
}

/*	------------------------------------------------------------------------------------------------
	
	------------------------------------------------------------------------------------------------	*/

void spi_set_buffer(uint8_t *buff, uint8_t size)
{
	spi_io.byte_array = buff;
	spi_io.size_byte_array = size;
	spi_io.byte_count = 0;
}

void spi_master_init()
{
	// MOSI	PA1/PC2
	// MISO	PA2/PC1 -	SPI controlled
	// SCK	PA3/PC0	-	User defined
	// SS	PA4/PC3	-	User defined
	// For for multiple master configuration, SS must be set as input and held high for master operation.
	
	PORTC.DIRCLR = PIN1_bm;
	PORTC.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm; // Set SCK, MOSI and SS as outputs (Alternate pins). SCK is also onboard LED for eval board.
	VPORTC.OUT &= ~PIN0_bm; // Set SCK low
	spi_slave_deselect();
	
	// Setup for non-buffer mode.
	SPI0.CTRLB = 0<<SPI_BUFEN_bp | 
				1<<SPI_BUFWR_bp | 
				1<<SPI_SSD_bp | 
				SPI_MODE_0_gc; // Disable multi-master for now
	SPI0.INTCTRL = SPI_IE_bm; // Buffer mode disabled, use this interrupt.	
	SPI0.CTRLA = 0<<SPI_DORD_bp | 
				1<<SPI_MASTER_bp | 
				0<<SPI_CLK2X_bp | 
				SPI_PRESC_DIV64_gc | 
				1<<SPI_ENABLE_bp; // MSB first as per MCP23S17 datasheet
}

void spi_start ()
{
	spi_slave_select();
	spi_io.byte_count = 0;
	spi_isr();
}