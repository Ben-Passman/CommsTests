#include "i2c_master.h"

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
	
	// !!SHOULD RETURN SUCCESS/FAIL!!
}

// Bus is IDLE, OWNER, BUSY or UNDEFINED. Errors are ARBLOST and BUSERR
// ARBLOST fails at any point on packet transmission, start -> NACK
// For ARBLOST must restart (write to MADDR). MADDR write clears ARBLOST flag.
// BUSERR is an illegal condition (e.g. Start followed by Stop). MADDR write clears BUSERR flag.

i2c_states_t i2c_start (register8_t addr, register8_t read_write) {
	// Unknown bus state will cause WIF and BUSERR to be set and operation will be terminated.
	// Bus is busy master will wait until idle.
	// On receiving ACK/NACK (if arbitration not lost) SCL held low and CLKHOLD and WIF set.
	// If bus already owned, repeat start generated. (ACK/NACK sent first if previous operation was read)
	// For read command RIF is set after byte received.
	// Writes to MSTATUS clear flags (CHECK).
	TWI0.MADDR = (addr<<1 | read_write);
	return I2C_BUSY;
}

i2c_states_t i2c_write (register8_t data) {
	// Read/Write only available during CLKHOLD.
	// On ACK/NACK receipt WIF set regardless of error. 
	// Check ARBLOST in multi-master environment.
	// Write to MDATA forces transmit, ignores ACKACT
	// MDATA read/write clears interrupt flags, but not ARBLOST or BUSERR
	// Read triggers ACK/NACK and one byte read. RIF then set, WIF on error.
	TWI0.MDATA = data;
	return I2C_BUSY;
}

i2c_states_t i2c_stop()
{
	// Executes ACK/NACK followed by stop condition
	TWI0.MCTRLB	|= TWI_MCMD_STOP_gc; // Set ACK/NACK as well for read
	return I2C_IDLE;
}