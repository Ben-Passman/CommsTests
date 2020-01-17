#include "i2c_master.h"

// BAUD = (F_CLK_PER/F_SCL - 10 - F_CLK_PER*T_RISE)/2;
#define I2C_BAUD 0x0B

typedef enum I2C_states_enum
{
	I2C_IDLE = 0,
	I2C_START,
	I2C_RESTART,
	I2C_STOP,
	I2C_RESET,
	I2C_ADDR_ACK,
	I2C_TX_BYTE,
	I2C_RX_BYTE,
	I2C_ARB_LOST,
	I2C_BUS_ERR
} i2c_states_t;

//typedef void(*callback)(void);
typedef i2c_states_t(*fsm_function)(void);
typedef i2c_operations_t(*event_callback)(void);

struct i2c_data
{
	uint8_t slave_addr;
	uint8_t *byte_array;
	uint8_t size_byte_array;
	uint8_t byte_count;
	i2c_states_t state;
	event_callback event_callbacks[6];
};

static struct i2c_data i2c_fsm = {.slave_addr = 0, .byte_array = NULL, .size_byte_array = 0, .byte_count = 0, .state = I2C_IDLE};

/*	------------------------------------------------------------------------------------------------

	------------------------------------------------------------------------------------------------	*/

static void i2c_send_ack()
{
	TWI0.MCTRLB &= ~TWI_ACKACT_bm; // ACK
	TWI0.MCTRLB |= TWI_MCMD_RECVTRANS_gc;
}

static void i2c_send_nack()
{
	TWI0.MCTRLB |= TWI_ACKACT_bm | TWI_MCMD_RECVTRANS_gc;
}

static void i2c_read_data()
{
	
	*(i2c_fsm.byte_array + i2c_fsm.byte_count) = TWI0.MDATA; // Read should trigger bus operation
	i2c_fsm.byte_count++;
}

static void i2c_write_data()
{
	TWI0.MDATA = *(i2c_fsm.byte_array + i2c_fsm.byte_count); // Write should trigger bus operation
	i2c_fsm.byte_count++;
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
		
	return I2C_ADDR_ACK;
}

static i2c_states_t I2C_M_RESTART(void)
{
	/*i2c_fsm.byte_count = 0;
	TWI0.MCTRLB |= TWI_MCMD_REPSTART_gc | TWI_ACKACT_bm;
			
	return I2C_ADDR_ACK;*/
	
	return I2C_M_START(); // Always update MADDR on restart
}

static i2c_states_t I2C_M_STOP(void)
{
	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
	
	return I2C_IDLE;
}

static i2c_states_t I2C_M_RESET(void)
{
	TWI0.MCTRLB |= TWI_FLUSH_bm;
	//TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	
	return I2C_IDLE;
}

static i2c_states_t i2c_M_callback_handler(i2c_events_t event)
{
	i2c_states_t next_state = i2c_fsm.state;
	i2c_operations_t next_op = i2c_fsm.event_callbacks[event]();

	switch(next_op)
	{
		case reset_i2c :
			next_state = I2C_M_RESET();
			break;
		case stop_i2c :
			next_state = I2C_M_STOP();
			break;
		case restart_i2c :
		default :
			next_state = I2C_M_RESTART();
			break;
	}

	return next_state;
}

static i2c_states_t I2C_M_ADDR_ACK(void)
{
	i2c_states_t next_state = I2C_IDLE;
	
	if (TWI0.MSTATUS & TWI_RXACK_bm) // ADDR NACK
	{
		next_state = i2c_M_callback_handler(address_NACK_error);
	}
	else if (TWI0.MSTATUS & TWI_RIF_bm)
	{
		i2c_read_data();	// READ IN FIRST BYTE
		i2c_send_ack();		// SEND ACK
		next_state = I2C_RX_BYTE;	// WAIT FOR NEXT BYTE
	}
	else if (TWI0.MSTATUS & TWI_WIF_bm)
	{
		i2c_write_data();	// SEND FIRST BYTE
		next_state = I2C_TX_BYTE;	// WAIT FOR ACK
	}
	
	return next_state;
}

static i2c_states_t I2C_M_TX_BYTE(void)
{
	i2c_states_t next_state = I2C_TX_BYTE;
	
	if (TWI0.MSTATUS & TWI_RXACK_bm) // DATA NACK
	{
		next_state = i2c_M_callback_handler(data_NACK_error);
	}
	else if(i2c_fsm.byte_count < i2c_fsm.size_byte_array)
	{
		i2c_write_data();	// SEND NEXT BYTE
	}
	else
	{
		next_state = i2c_M_callback_handler(tx_complete);
	}
	
	return next_state;
}

static i2c_states_t I2C_M_RX_BYTE(void)
{
	i2c_states_t next_state = I2C_RX_BYTE;
	
	i2c_read_data();	// READ BYTE INTO BUFFER
	
	if (i2c_fsm.byte_count < i2c_fsm.size_byte_array)
	{
		i2c_send_ack();
	}
	else
	{
		i2c_send_nack();
		next_state = i2c_M_callback_handler(rx_complete);
	}
	
	return next_state;
}

// ARBITRATION ERROR
// Can lose arbitration during high data, NACK or start/repeat start.
// Need to abort or re-send start (MADDR). Write to MADDR will clear flag.
static i2c_states_t I2C_M_ARB_LOST(void)
{
	return i2c_M_callback_handler(ARB_error);
}

// BUS ERROR
// S/Sr/P violation
// Reset peripheral and Re-send. MADDR clears BUSERR flag.
static i2c_states_t I2C_M_BUS_ERR(void)
{
	return i2c_M_callback_handler(bus_error);
}


static fsm_function state_callbacks[] = {
	I2C_M_IDLE,
	I2C_M_START,
	I2C_M_RESTART,
	I2C_M_STOP,
	I2C_M_RESET,
	I2C_M_ADDR_ACK,
	I2C_M_TX_BYTE,
	I2C_M_RX_BYTE,
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
	if (TWI0.MSTATUS & TWI_ARBLOST_bm)
	{
		i2c_fsm.state = I2C_ARB_LOST;
	}
	if (TWI0.MSTATUS & TWI_BUSERR_bm)
	{
		i2c_fsm.state = I2C_BUS_ERR;
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
	Default Callbacks
	
	Don't do anything other than determine the next state for the FSM.
	------------------------------------------------------------------------------------------------	*/

	i2c_operations_t stop_cb (void)
	{
		return stop_i2c;
	}
	
	i2c_operations_t restart_cb (void)
	{
		return restart_i2c;
	}

/*	------------------------------------------------------------------------------------------------

	------------------------------------------------------------------------------------------------	*/

void i2c_master_init()
{
	// Peripheral controls
	TWI0.CTRLA = TWI_SDASETUP_4CYC_gc | TWI_SDAHOLD_OFF_gc | 0<<TWI_FMPEN_bp;
	TWI0.DBGCTRL = TWI_DBGRUN_bm; // Keep peripheral running during debug mode
	// I2C Master controls
	TWI0.MBAUD = I2C_BAUD; // Derives SCL hi/lo times. (See data sheet for equation) Write while ENABLE bit in TWI.MCTRLA is 0
	TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_TIMEOUT_DISABLED_gc | TWI_ENABLE_bm; // Quick command and smart mode not in use
	TWI0.MCTRLB &= 0<<TWI_ACKACT_bp; // 0 send ACK, 1 send NACK when data read or execute command written to CMD bits
	TWI0_MCTRLB |= TWI_FLUSH_bm;
	
	// ASSUMES BUS IS IDLE, SHOULD CHECK SDA/SCK LINES. (Port A 1&2 for alt. pin config)
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;	
	
	// Points in the FSM where external input may be required
	i2c_fsm.event_callbacks[tx_complete] = stop_cb;
	i2c_fsm.event_callbacks[rx_complete] = stop_cb;
	i2c_fsm.event_callbacks[address_NACK_error] = stop_cb;
	i2c_fsm.event_callbacks[data_NACK_error] = stop_cb;
	i2c_fsm.event_callbacks[ARB_error] = restart_cb;
	i2c_fsm.event_callbacks[bus_error] = restart_cb;

	i2c_fsm.state = I2C_IDLE;
}

void i2c_set_event_callback(i2c_events_t ev, event_callback cb)
{
	i2c_fsm.event_callbacks[ev] = cb;
}

void i2c_set_buffer(uint8_t slave_addr, uint8_t *data, uint8_t byte_count)
{
	i2c_fsm.slave_addr = slave_addr;
	i2c_fsm.byte_array = data;
	i2c_fsm.size_byte_array = byte_count;
}

void i2c_start ()
{
	i2c_fsm.state = I2C_START;
	i2c_state_isr();
}

uint8_t i2c_idle ()
{
	return i2c_fsm.state == I2C_IDLE;
}