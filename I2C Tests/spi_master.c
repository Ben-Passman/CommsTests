#include "spi_master.h"
#include <avr/interrupt.h>

typedef enum SPI_states_enum
{
	SPI_BUSY = 0,
	SPI_TRANSFER_COMPLETE,
	SPI_COL_ERR,
	SPI_IDLE
} spi_states_t;

typedef spi_states_t (*spi_cb) (void);

typedef struct
{
	spi_states_t status;
	const uint8_t *tx_byte_array;
	uint8_t *rx_byte_array;
	uint8_t size_byte_array;	
	uint8_t byte_count;
	cb buff_full_cb;
} spi_data;

static spi_data spi_io = {0};

spi_states_t spi_slave_select(void)
{
	VPORTC.OUT &= ~PIN3_bm; // Set SS low
	return SPI_BUSY;
}

spi_states_t spi_slave_deselect(void)
{
	VPORTC.OUT |= PIN3_bm;
	return spi_io.status;
}

spi_states_t spi_slave_reset(void)
{
	spi_slave_deselect();
	return SPI_IDLE;
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
	*(spi_io.rx_byte_array + spi_io.byte_count) = SPI0.DATA;
	spi_io.byte_count ++;
	
	if (spi_io.byte_count < spi_io.size_byte_array)
	{
		SPI0.DATA = *(spi_io.tx_byte_array + spi_io.byte_count);		
	}
	else
	{
//		spi_io.callback[SPI_TRANSFER_COMPLETE];
//		spi_io.buff_full_cb();
		spi_io.status = spi_slave_reset();

	}
}

ISR(SPI0_INT_vect)
{
	register8_t int_flags = SPI0.INTFLAGS;
	// SPI_WRCOL_bm		write collision
	// SPI_IF_bm		byte transfer complete
	
	// CURRENTLY NO COLLISION ERROR HANDLING
	if (int_flags & SPI_WRCOL_bm) 
	{
//		spi_io.status = SPI_COL_ERR;
//		spi_io.callback[SPI_COL_ERR];
		spi_io.status = spi_slave_reset();
	} 
	else
	{
		spi_isr();
	}
	
	SPI0.INTFLAGS = 0x00;
}

/*	------------------------------------------------------------------------------------------------
	
	------------------------------------------------------------------------------------------------	*/

void spi_master_init()
{
	// For for multiple master configuration, SS must be set as input and held high for master operation.
	
	PORTC.DIRCLR = PIN1_bm;
	PORTC.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm; // Set SCK, MOSI and SS as outputs (Alternate pins). SCK is also onboard LED for eval board.
	VPORTC.OUT &= ~PIN0_bm; // Set SCK low
	spi_io.status = spi_slave_reset();
/*	spi_io.callback[SPI_BUSY] = spi_slave_deselect;
	spi_io.callback[SPI_TRANSFER_COMPLETE] = spi_slave_deselect;
	spi_io.callback[SPI_COL_ERR] = spi_slave_deselect;*/
	
	// Setup for non-buffer mode.
	SPI0.CTRLB = 0<<SPI_BUFEN_bp | 
				1<<SPI_BUFWR_bp | 
				1<<SPI_SSD_bp | 
				SPI_MODE_0_gc; // Disable multi-master for now
	SPI0.INTCTRL = SPI_IE_bm; // Buffer mode disabled, use this interrupt.	
	SPI0.CTRLA = 0<<SPI_DORD_bp | // Transmit MSB first
				1<<SPI_MASTER_bp | 
				0<<SPI_CLK2X_bp | 
				SPI_PRESC_DIV64_gc | 
				1<<SPI_ENABLE_bp;
}

void spi_start (const uint8_t *tx_buff, uint8_t *rx_buff, uint8_t size, cb callback)
{
	switch(spi_io.status)
	{
		case SPI_IDLE:
			// No multi-master -> no check that bus already in use
			spi_io.status = spi_slave_select();
			spi_io.tx_byte_array = tx_buff;
			spi_io.rx_byte_array = rx_buff;
			spi_io.size_byte_array = size;
			spi_io.byte_count = 0;
			spi_io.buff_full_cb = callback;
			SPI0.DATA = *spi_io.tx_byte_array;
			break;
		default:
			break;
	}
}