#include "i2c_master.h"

typedef enum I2C_states_enum
{
	I2C_IDLE = 0,
	I2C_START,
	I2C_STOP,
	I2C_RESET,
	I2C_TX_BYTE,
	I2C_RX_BYTE,
	I2C_NACK,
	I2C_ARB_ERR,
	I2C_BUS_ERR
} i2c_states_t;

typedef enum I2C_status_enum
{
	TX_COMPLETE = 0,
	RX_COMPLETE,
	NACK_ERROR,
	ARB_ERROR,
	BUS_ERROR
} i2c_status_t;

typedef i2c_states_t(*fsm_function)(void);

struct i2c_data
{
	uint8_t slave_addr;
	uint8_t *byte_array;
	uint8_t size_byte_array;
	uint8_t byte_count;
	i2c_states_t state;
//	fsm_function state_handler[5];
};

static struct i2c_data i2c_fsm = {.slave_addr = 0, .byte_array = NULL, .size_byte_array = 0, .byte_count = 0, .state = I2C_IDLE};

void i2c_set_buffer(uint8_t addr, uint8_t read_write, uint8_t *data, uint8_t data_size)
{
	i2c_fsm.slave_addr = addr<<1 | read_write;
	i2c_fsm.byte_array = data;
	i2c_fsm.size_byte_array = data_size;
}

/*	------------------------------------------------------------------------------------------------
	FSM FUNCTIONS
	------------------------------------------------------------------------------------------------	*/

static i2c_states_t I2C_M_IDLE(void)
{
	return I2C_IDLE;
}

static i2c_states_t I2C_M_START(void)
{
	// Unknown bus state will cause WIF and BUSERR to be set and operation will be terminated.
	// Bus is busy master will wait until idle.
	// On receiving ACK/NACK (if arbitration not lost) SCL held low and CLKHOLD and WIF set.
	// If bus already owned, repeat start generated. (ACK/NACK sent first if previous operation was read)
	// For read command RIF is set after byte received.
	// Writes to MSTATUS clear flags (CHECK).
	
	i2c_fsm.byte_count = 0;
	TWI0.MADDR = i2c_fsm.slave_addr;
		
	return (i2c_fsm.slave_addr & I2C_READ_bm) ? I2C_RX_BYTE : I2C_TX_BYTE;
}

/*static i2c_states_t I2C_M_RESTART(void)
{
	// Set buffer
	//return I2C_M_START();
}*/

static i2c_states_t I2C_M_STOP(void)
{
	// NACK needs to be set to end RX transmission. Should do here...
	TWI0.MCTRLB	|= TWI_MCMD_STOP_gc;
	
	return I2C_IDLE;
}

static i2c_states_t I2C_M_RESET(void)
{
	TWI0.MCTRLB |= TWI_FLUSH_bm;
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	
	return I2C_IDLE;
}

static i2c_states_t I2C_M_TX(void)
{
	// MDATA Read/Write only available during CLKHOLD.
	// Write to MDATA forces transmit, ignores ACKACT
	// MDATA read/write clears interrupt flags, but not ARBLOST or BUSERR
	TWI0.MDATA = *(i2c_fsm.byte_array + i2c_fsm.byte_count);
	i2c_fsm.byte_count ++;
	
	// ACK from slave will trigger next interrupt
	return (i2c_fsm.byte_count >= i2c_fsm.size_byte_array) ? I2C_STOP : I2C_TX_BYTE;
}

static i2c_states_t I2C_M_RX(void)
{
	i2c_states_t output = I2C_RX_BYTE;
	*(i2c_fsm.byte_array + i2c_fsm.byte_count) = TWI0.MDATA;
	
	// To send NACK, set ACKACT in MCTRLB to 1
	if (i2c_fsm.byte_count < i2c_fsm.size_byte_array)
	{
		TWI0.MCTRLB &= ~TWI_ACKACT_bm; // Send ACK
		TWI0.MCTRLB |= TWI_MCMD_RECVTRANS_gc; // Need to manually trigger this.
	}
	else
	{
		// STOP condition requires NACK to pull SDA line low.
		TWI0.MCTRLB |= TWI_ACKACT_bm; // Send NACK
		output = I2C_M_STOP();
	}
	
	i2c_fsm.byte_count ++;
	
	return output;
}

static i2c_states_t I2C_M_RX_NACK(void)
{
	// No distinction between DATA/ADDR fail
	return I2C_M_STOP();
}

// ARBITRATION ERROR
// Can lose arbitration during high data, NACK or start/repeat start.
// Need to abort or re-send start (MADDR). Write to MADDR will clear flag.
static i2c_states_t I2C_M_ARB_LOST(void)
{
	return I2C_M_STOP();
}

// BUS ERROR
// S/Sr/P violation
// Reset peripheral and Re-send. MADDR clears BUSERR flag.
static i2c_states_t I2C_M_BUS_ERR(void)
{
	return I2C_M_RESET();
}

static fsm_function state_callbacks[] = {
	I2C_M_IDLE,
	I2C_M_START,
	I2C_M_STOP,
	I2C_M_RESET,
	I2C_M_TX,
	I2C_M_RX,
	I2C_M_RX_NACK,
	I2C_M_ARB_LOST,
	I2C_M_BUS_ERR
};

static void i2c_state_isr (void)
{
	// RIF set on master read if no errors (ARB/BUSERR)
	// WIF set on write completion, regardless of error
	// Check ARBLOST in multi-master environment.
	// Read triggers ACK/NACK and one byte read. RIF then set, WIF on error.
	// TWI0.MSTATUS |= TWI_WIF_bm | TWI_RIF_bm; // Not required. Cleared by state machine operations.	
	
	if (TWI0.MSTATUS & TWI_RXACK_bm) // NACK received
	{
		i2c_fsm.state = I2C_STOP;
	}
	
	if (TWI0.MSTATUS & TWI_ARBLOST_bm)
	{
		i2c_fsm.state = I2C_STOP;
	}
	
	if(TWI0.MSTATUS & TWI_BUSERR_bm)
	{
		i2c_fsm.state = I2C_RESET;
	}
	
	i2c_fsm.state = state_callbacks[i2c_fsm.state]();
}

/*	------------------------------------------------------------------------------------------------
	INTERRUPTS
	------------------------------------------------------------------------------------------------	*/

ISR(TWI0_TWIM_vect)
{
	// Bus is IDLE, OWNER, BUSY or UNDEFINED. Errors are ARBLOST and BUSERR
	// ARBLOST fails at any point on packet transmission, start -> NACK
	// For ARBLOST must restart (write to MADDR). MADDR write clears ARBLOST flag.
	// BUSERR is an illegal condition (e.g. Start followed by Stop). MADDR write clears BUSERR flag.
	
	i2c_state_isr();
}

/*	------------------------------------------------------------------------------------------------
	
	------------------------------------------------------------------------------------------------	*/

void i2c_master_init()
{
	// Peripheral controls
	TWI0.CTRLA = TWI_SDASETUP_4CYC_gc | TWI_SDAHOLD_OFF_gc | 0<<TWI_FMPEN_bp;
	TWI0.DBGCTRL = TWI_DBGRUN_bm; // Keep peripheral running during debug mode
	// I2C Master controls
	TWI0.MBAUD = BAUD; // Derives SCL hi/lo times. (See data sheet for equation) Write while ENABLE bit in TWI.MCTRLA is 0
	TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_TIMEOUT_DISABLED_gc | TWI_ENABLE_bm; // Quick command and smart mode not in use
	TWI0.MCTRLB &= 0<<TWI_ACKACT_bp; // 0 send ACK, 1 send NACK when data read or execute command written to CMD bits
	TWI0_MCTRLB |= TWI_FLUSH_bm;
	
	// ASSUMES BUS IS IDLE, SHOULD CHECK SDA/SCK LINES. (Port A 1&2 for alt. pin config)
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;	

	i2c_fsm.state = I2C_IDLE;
	
	// SHOULD RETURN SUCCESS/FAIL?
}

void i2c_start (uint8_t addr, uint8_t read_write, uint8_t *data, uint8_t data_size) {
	// Also need to check bus is free for multi-master.
	if (i2c_fsm.state == I2C_IDLE)
	{
		i2c_set_buffer(addr, read_write, data, data_size);
		i2c_fsm.state = I2C_START;
		
		i2c_state_isr();
	}
}