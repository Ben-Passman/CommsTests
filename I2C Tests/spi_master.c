#include "spi_master.h"

struct spi_data
{
	spi_status_t status;
	const uint8_t *tx_byte_array;
	uint8_t *rx_byte_array;
	uint8_t size_byte_array;	
	uint8_t byte_count;
};

static struct spi_data spi_io = {.status = SPI_IDLE, .tx_byte_array = NULL, .rx_byte_array = NULL, .size_byte_array = 0, .byte_count = 0};

static spi_status_t spi_slave_select(void)
{
	VPORTC.OUT &= ~PIN3_bm; // Set SS low
	return SPI_BUSY;
}

static spi_status_t spi_slave_deselect(void)
{
	VPORTC.OUT |= PIN3_bm;
	return spi_io.status;
}

static spi_status_t spi_slave_reset(void)
{
	spi_slave_deselect();
	return SPI_IDLE;
}

/*	------------------------------------------------------------------------------------------------
	INTERRUPTS
	------------------------------------------------------------------------------------------------	*/

static void spi_isr (void)
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

spi_status_t spi_master_init()
{
	// For for multiple master configuration, SS must be set as input and held high for master operation.
	
	PORTC.DIRCLR = PIN1_bm;
	PORTC.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm; // Set SCK, MOSI and SS as outputs (Alternate pins). SCK is also onboard LED for eval board.
	VPORTC.OUT &= ~PIN0_bm; // Set SCK low
	spi_io.status = spi_slave_reset();
	
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
				
	return spi_io.status;
}

spi_status_t spi_start (const uint8_t *tx_buff, uint8_t *rx_buff, uint8_t byte_count)
{
	spi_status_t spi_state = SPI_IDLE;
	switch(spi_io.status)
	{
		case SPI_IDLE:
			// No multi-master -> no check that bus already in use
			spi_io.status = spi_slave_select();
			spi_io.tx_byte_array = tx_buff;
			spi_io.rx_byte_array = rx_buff;
			spi_io.size_byte_array = byte_count;
			spi_io.byte_count = 0;
			SPI0.DATA = *spi_io.tx_byte_array;
			spi_state = spi_io.status;
			break;
		default:
			spi_io.status = SPI_TX_BUFF_ERR;
			break;
	}
	return spi_state;
}

spi_status_t get_spi_status (void)
{
	return spi_io.status;
}