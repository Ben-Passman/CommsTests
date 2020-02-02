#include "spi_master.h"
	
#define SPI_SLAVE_SELECT VPORTC.OUT &= ~PIN3_bm;
#define SPI_SLAVE_DESELECT VPORTC.OUT |= PIN3_bm;

struct spi_data
{
	uint8_t spi_busy : 1; // Position unimportant. Can use bit field.
	uint8_t spi_error : 1;
	const uint8_t *tx_byte_array;
	uint8_t *rx_byte_array;
	uint8_t size_byte_array;	
	uint8_t byte_count;
	spi_event_callback transmit_complete_callback;
	spi_event_callback error_callback;
};

static struct spi_data spi_io = {.spi_busy = 0, .spi_error = 0, .tx_byte_array = NULL, .rx_byte_array = NULL, .size_byte_array = 0, .byte_count = 0};

/*	------------------------------------------------------------------------------------------------
	
	------------------------------------------------------------------------------------------------	*/

static void spi_m_start(void)
{
	spi_io.spi_busy = 1;
	spi_io.byte_count = 0;
	SPI_SLAVE_SELECT;
	SPI0.DATA = *spi_io.tx_byte_array;	
}

static void spi_m_stop(void)
{
	SPI_SLAVE_DESELECT;
	spi_io.spi_busy = 0;
}

static void spi_m_reset(void)
{
	// Additional code here...
	spi_m_stop();	
}

static void spi_m_callback_handler(spi_event_callback cb)
{
	switch(cb())
	{
		case stop_spi :
			spi_m_stop();
			break;
		case restart_spi :
			spi_m_start();
			break;
		case reset_spi :
			spi_m_reset();
			break;
	}
}

static void spi_m_transmit(void)
{
	*(spi_io.rx_byte_array + spi_io.byte_count) = SPI0.DATA;
	spi_io.byte_count ++;
	
	if (spi_io.byte_count < spi_io.size_byte_array)
	{
		SPI0.DATA = *(spi_io.tx_byte_array + spi_io.byte_count);
	}
	else
	{
		spi_m_callback_handler(spi_io.transmit_complete_callback);
	}
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
	if (SPI0.INTFLAGS & SPI_WRCOL_bm)
	{
		spi_io.spi_error = 1;
		spi_m_callback_handler(spi_io.error_callback);
	}
	else
	{
		spi_m_transmit();
	}
	
}

ISR(SPI0_INT_vect)
{
	spi_isr();
}

/*	------------------------------------------------------------------------------------------------
	CALLBACK PLACEHOLDER FUNCTIONS
	
	Don't do anything other than specify which action to take.
	------------------------------------------------------------------------------------------------	*/

spi_command_t spi_stop_cb(void)
{
	return stop_spi;
}

spi_command_t spi_reset_cb(void)
{
	return reset_spi;
}

/*	------------------------------------------------------------------------------------------------
	
	------------------------------------------------------------------------------------------------	*/

void spi_master_init()
{
	// For for multiple master configuration, SS must be set as input and held high for master operation.
	
	PORTC.DIRCLR = PIN1_bm;
	PORTC.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm; // Set SCK, MOSI and SS as outputs (Alternate pins). SCK is also onboard LED for eval board.
	VPORTC.OUT &= ~PIN0_bm; // Set SCK low
	spi_m_reset();
	spi_io.transmit_complete_callback = spi_stop_cb;
	spi_io.error_callback = spi_reset_cb;
	spi_io.spi_busy = 0;
	
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

void spi_start (const uint8_t *tx_buff, uint8_t *rx_buff, uint8_t byte_count)
{
	// No multi-master -> no check that bus already in use
	spi_io.tx_byte_array = tx_buff;
	spi_io.rx_byte_array = rx_buff;
	spi_io.size_byte_array = byte_count;
	spi_m_start();
}

void set_transmit_complete_callback(spi_event_callback cb)
{
	spi_io.transmit_complete_callback = cb;
}

void set_data_error_callback(spi_event_callback cb)
{
	spi_io.error_callback = cb;
}

uint8_t spi_idle (void)
{
	return !spi_io.spi_busy;
}