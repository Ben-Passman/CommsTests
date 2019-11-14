#include "i2c_master.h"
#include <avr/interrupt.h>

typedef enum I2C_states_enum
{
	I2C_IDLE = 0,
	I2C_BUSY,
	I2C_ERR,
	I2C_START,
	I2C_STOP,
	I2C_TX_BYTE,
	I2C_RX_BYTE
} i2c_states_t;

typedef struct
{
	uint8_t slave_addr;
	uint8_t *byte_array;
	uint8_t size_byte_array;
	uint8_t byte_count;
	i2c_states_t state;
} I2C_data;

typedef i2c_states_t(*state_function_array)(void);

I2C_data setup_data = {0};

void i2c_set_buffer(uint8_t *buff, uint8_t size) 
{
	setup_data.byte_array = buff;
	setup_data.size_byte_array = size;
	setup_data.byte_count = 0;
}

void i2c_master_init()
{
	// Peripheral controls
	TWI0.CTRLA = TWI_SDASETUP_4CYC_gc | TWI_SDAHOLD_OFF_gc; // FMPEN (fast mode) set to zero
	TWI0.DBGCTRL = TWI_DBGRUN_bm; // Keep peripheral running during debug mode
	// I2C Master controls
	TWI0.MBAUD = 0x0B; // Derives SCL hi/lo times. (See data sheet for equation) Write while ENABLE bit in TWI.MCTRLA is '0
	TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_TIMEOUT_DISABLED_gc | TWI_ENABLE_bm; // Quick command and smart mode not in use
	TWI0.MCTRLB &= ~TWI_ACKACT_bm; // 0 send ACK, 1 send NACK when data read or execute command written to CMD bits
	TWI0_MCTRLB |= TWI_FLUSH_bm;
	
	// !!ASSUMES BUS IS IDLE, SHOULD CHECK LINES ARE GOOD!!
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	setup_data.state = I2C_IDLE;
	
	// SHOULD RETURN SUCCESS/FAIL?
}

void i2c_start (uint8_t addr, uint8_t read_write) {
	setup_data.byte_count = 0;
	setup_data.slave_addr = addr;
	setup_data.state = (read_write == I2C_WRITE_bm) ?  I2C_TX_BYTE : I2C_RX_BYTE;
	
	TWI0.MADDR = (addr<<1 | read_write);
}

/*	------------------------------------------------------------------------------------------------
	FSM FUNCTIONS
	------------------------------------------------------------------------------------------------	*/

static i2c_states_t I2C_M_PLACEHOLDER()
{
	return I2C_IDLE;
}

static i2c_states_t I2C_M_START()
{
	// Unknown bus state will cause WIF and BUSERR to be set and operation will be terminated.
	// Bus is busy master will wait until idle.
	// On receiving ACK/NACK (if arbitration not lost) SCL held low and CLKHOLD and WIF set.
	// If bus already owned, repeat start generated. (ACK/NACK sent first if previous operation was read)
	// For read command RIF is set after byte received.
	// Writes to MSTATUS clear flags (CHECK).
	setup_data.byte_count = 0;
	TWI0.MADDR = (setup_data.slave_addr<<1 | I2C_WRITE_bm);
	
	// READ/WRITE SELECT NOT YET IMPLEMENTED.	
	return (I2C_WRITE_bm == I2C_WRITE_bm) ? I2C_TX_BYTE : I2C_RX_BYTE;
}

static i2c_states_t I2C_M_STOP()
{
	// Executes ACK/NACK followed by stop condition. Set ACK/NACK as well for read.
	TWI0.MCTRLB	|= TWI_MCMD_STOP_gc;
	
	return I2C_IDLE;
}


static i2c_states_t I2C_M_WRITE()
{
	// Read/Write only available during CLKHOLD.
	// On ACK/NACK receipt WIF set regardless of error. 
	// Check ARBLOST in multi-master environment.
	// Write to MDATA forces transmit, ignores ACKACT
	// MDATA read/write clears interrupt flags, but not ARBLOST or BUSERR
	// Read triggers ACK/NACK and one byte read. RIF then set, WIF on error.
	
	TWI0.MDATA = *(setup_data.byte_array + setup_data.byte_count);
	setup_data.byte_count ++;
	
	return (setup_data.byte_count >= setup_data.size_byte_array) ? I2C_STOP : I2C_TX_BYTE;
}

state_function_array state_callbacks[] = {
	I2C_M_PLACEHOLDER,
	I2C_M_PLACEHOLDER,
	I2C_M_PLACEHOLDER,
	I2C_M_START,
	I2C_M_STOP,
	I2C_M_WRITE,
	I2C_M_PLACEHOLDER
};

/*	------------------------------------------------------------------------------------------------
	INTERRUPTS
	------------------------------------------------------------------------------------------------	*/

ISR(TWI0_TWIM_vect)
{
	// Bus is IDLE, OWNER, BUSY or UNDEFINED. Errors are ARBLOST and BUSERR
	// ARBLOST fails at any point on packet transmission, start -> NACK
	// For ARBLOST must restart (write to MADDR). MADDR write clears ARBLOST flag.
	// BUSERR is an illegal condition (e.g. Start followed by Stop). MADDR write clears BUSERR flag.
	
	// Debug, count i2c ops and light up LEDs
	VPORTA.OUT |= 0x10 << setup_data.byte_count;
	
	// NEED ERROR CHECKS HERE, BUSERR + ARBLOST + NACK.
	// (TWI0.MSTATUS & TWI_RXACK_bm)
	setup_data.state = state_callbacks[setup_data.state]();
}