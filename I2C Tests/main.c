/*
 * I2C Tests.c
 *
 * Created: 13/10/2019 10:34:51 AM
 * Author : Ben
 */ 

#define SLAVE1_ADDR 0x20
#define MCP23017_READ 0x01
#define MCP23017_WRITE 0x00

/* MCP23X17 Registers (IOCON BANK bit = 0) */
typedef enum MCP23X17_REG_enum
{
	IODIRA = 0x00,		// I/O Direction, 1=input
	IPOLA = 0x02,		// Input polarity, 0=same
	GPINTENA = 0x04,	// Interrupt-on-change enable, 0=disabled
	DEFVALA = 0x06,		// Default pin value
	INTCONA = 0x08,		// Interrupt control, 0=previous state (1=DEFVALA)
	IOCON = 0x0A,		// GPIO control
	GGPUA = 0x0C,		// Input pull-up, 0=disabled
	INTFA = 0x0E,		// Interrupt flags
	INTCAPA = 0x10,		// Port status at time of interrupt
	GPIOA = 0x12,		// Pin values
	OLATA = 0x14,		// Output pin latches
	IODIRB = 0x01,
	IPOLB = 0x03,
	GPINTENB = 0x05,
	DEFVALB = 0x07,
	INTCONB = 0x09,
	IOCONB = 0x0B,		// Equivalent to IOCON
	GGPUB = 0x0D,
	INTFB = 0x0F,
	INTCAPB = 0x11,
	GPIOB = 0x13,
	OLATB = 0x15,
} MCP23X17_REG_t;

/* IOCON bit masks and bit positions */
#define IOCON_BANK_bm 0x40
#define IOCON_BANK_bp 7
#define IOCON_MIRROR_bm 0x40
#define IOCON_MIRROR_bp 6
#define IOCON_SEQOP_bm 0x40
#define IOCON_SEQOP_bp 5
#define IOCON_DISSLW_bm 0x40
#define IOCON_DISSLW_bp 4
#define IOCON_HAEN_bm 0x40
#define IOCON_HAEN_bp 3
#define IOCON_ODR_bm 0x40
#define IOCON_ODR_bp 2
#define IOCON_INTPOL_bm 0x40
#define IOCON_INTPOL_bp 1

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>

#define F_CPU (20E6/2)
#define BAUD_RATE 57600

void usart_init()
{
	USART0.CTRLB = USART_TXEN_bm;
	USART0.BAUD = (F_CPU * 64.0) / (BAUD_RATE * 16.0);
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

/*
	ATTiny817
	- 8kB Flash		0x8000	->	Program storage
	- 128B EEPROM	0x1400	->	Data Memory
	- 512B SRAM		0x3E00	->	Data storage & stack
	
	From schematics:
	- LED turned on by driving PC0 high.
	- User button is low-active on PC5, with external pullup.
	
	Notes:
	- TWI0 uses MADDR and MDATA registers for data transfer. MADDR contains a 7-bit address and the R/W bit, MDATA contains 8 bits of data.
*/
/*struct i2c_packet {
	uint8_t i2c_opcode;
	uint8_t i2c_reg_addr;
	uint8_t data; // data [4]; // Array
};*/

typedef struct  
{
	uint8_t *byte_array;
	uint8_t size_byte_array;
	uint8_t byte_count;
} I2C_data;

// size_t n = sizeof(a) / sizeof(a[0]); // Does not work for function parameters. Save size_n as well.
/*struct MCP23X17_states
{
	uint8_t address;
	uint8_t def_val;
};
	
struct MCP23X17_states MCP23X17_setup[] = {
	{.address = IOCON, .def_val = IOCON_SEQOP_bm	},
	{.address = IODIRA, .def_val = 0x00	},
	{.address = OLATA, .def_val = 0x0F	}
};*/

uint8_t MCP_addr = 0x00;
// Need to initialise expander IO port, IODRA, 0x00
uint8_t bytes[] = {	OLATA, 0x67	};
I2C_data setup_data = {.byte_array = bytes, .size_byte_array = 2, .byte_count = 0};
uint8_t read_ptr = 0;

int main(void)
{
	// PORTC.DIR = PIN0_bm; // Set PIN0 as output (r/m/w) or use SET/CLR/TGL
	// PORTC.OUT to set output (r/m/w) or use SET/CLR/TGL
	PORTC.DIRSET = PIN0_bm; // Set PIN0 as output
	PORTC.PIN5CTRL = PORT_ISC_FALLING_gc; // Interrupt on button down
	PORTB.DIRSET = PIN2_bm | PIN3_bm;
	PORTA.DIRSET = PIN1_bm | PIN2_bm;
	//PORTA.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm; // Changed from pins 4,5,6 now I2C works?
	PORTA.DIRSET = PIN4_bm | PIN5_bm | PIN6_bm;
	usart_init();
	TCA0.SINGLE.PERL = 0xB7;
	TCA0.SINGLE.PERH = 0x0C;
	TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV1024_gc; 
	
	/*	SYSTEM CLOCK	-	Config change protected
	USE DEFAULTS. NO NEED TO MODIFY	*/
//	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc | CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm); // Use 16/20MHz internal oscillator. (FUSE.OSCCFG chooses 20/16)	
//	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm); // Set prescale to 6x (0x8<<1) -> 3.333MHz and enable
//	_PROTECTED_WRITE(CLKCTRL.MCLKLOCK, CLKCTRL_LOCKEN_bm); // Lock A&B registers until next hardware reset
//	CLKCTRL.MCLKSTATUS // read only register for clock sources -> CLKCTRL_OSC20MS_bm for stability
//	CLKCTRL.OSC20MCTRLA &= ~CLKCTRL_RUNSTDBY_bm;	// Turn off force oscillator on
	
//	CLKCTRL.OSC20MCALIBA -> fine tuning around centre frequency, factory calibrated values loaded on reset
//	CLKCTRL.OSC20MCALIBB -> lock bit and temperature compensation
	
	/*	I/O Lines	*/
	//PORTMUX.CTRLB &= ~PORTMUX_TWI0_bm; // Clear alternate pinout (Cleared by default)
	PORTMUX.CTRLB = PORTMUX_TWI0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc | PORTMUX_USART0_DEFAULT_gc;
		
	/*	Interrupts	*/
	SREG |= CPU_I_bm; // Enable interrupts
	
	/*	TWI0	*/
	TWI0.CTRLA = TWI_SDASETUP_4CYC_gc | TWI_SDAHOLD_OFF_gc; // FMPEN (fast mode) set to zero
// TWI0.DBGCTRL = TWI_DBGRUN_bm; // Keep peripheral running during debug mode
	/*	MASTER CONTROL REGISTERS	*/
	TWI0.MBAUD = 0x0B; // 0x0C // Derives SCL hi/lo times. (See data sheet for equation) Write while ENABLE bit in TWI.MCTRLA is '0
	TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_TIMEOUT_DISABLED_gc | TWI_ENABLE_bm; // Quick command and smart mode not in use
	//TWI0.MCTRLB |= TWI_FLUSH_bm; // generates 1 clock strobe, master enable/disable
	TWI0.MCTRLB &= ~TWI_ACKACT_bm; // 0 send ACK, 1 send NACK when data read or execute command written to CMD bits
	TWI0_MCTRLB |= TWI_FLUSH_bm;
	//TWI0.MCTRLB |= TWI_MCMD_NOACT_gc; // Command bits (strobes)
		//TWI_MCMD_REPSTART_gc -> Issue Repeated Start Condition
		//TWI_MCMD_RECVTRANS_gc -> Receive or Transmit Data, depending on DIR
		//TWI_MCMD_STOP_gc -> Issue stop condition
	//TWI0.MSTATUS -> status bits, RIF, WIF, CLKHOLD, RXACK, ARBLOST, BUSERR, BUSSTATE (1:0)
	while(PORTA.IN & (PIN1_bm | PIN2_bm)){}
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc; // ASSUMES BUS IS IDLE, NO CHECKS
	/*	SLAVE CONTROL REGISTERS	*/
	
	while (1)
    {
		
    }
}

ISR(PORTC_PORT_vect)
{
	uint8_t intflags = PORTC.INTFLAGS; // Get flags
	PORTC.INTFLAGS = intflags; // Clear flags
		
	PORTC.OUTTGL = PIN0_bm;
	setup_data.byte_count = 0;
	TWI0.MADDR = (SLAVE1_ADDR<<1 | MCP23017_WRITE);
	//TWI0.MADDR = (SLAVE1_ADDR<<1 | MCP23017_READ);
	TCA0.SINGLE.CTRLESET |= TCA_SPLIT_CMD_RESTART_gc;
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
	VPORTA.OUT &= ~(PIN4_bm | PIN5_bm | PIN6_bm);
	bytes[1] = bytes[1] >> 7 | bytes[1] << 1;
}

ISR(TWI0_TWIM_vect)
{
	register8_t status_LEDs = PORTA.OUT;
	register8_t tx_byte;
	VPORTA.OUT |= 0x10 << setup_data.byte_count;
	//status_LEDs = (TWI0.MSTATUS & TWI_RXACK_bm) ? status_LEDs | PIN4_bm : status_LEDs & ~PIN4_bm;
	//status_LEDs = (TWI0.MSTATUS & TWI_ARBLOST_bm) ? status_LEDs | PIN5_bm : status_LEDs & ~PIN5_bm;
	//status_LEDs = (TWI0.MSTATUS & TWI_BUSERR_bm) ? status_LEDs | PIN6_bm : status_LEDs & ~PIN6_bm;
	
//	TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm); // Clear interrupt flags. Should be cleared automatically by MDATA?
	
//	status_LEDs |= (TWI0.MSTATUS & TWI_RXACK_bm) >> (4 - setup_data.byte_count); // RXACK -> 0x10
//	VPORTA.OUT = status_LEDs; // VPORT allows for bit manipulation instructions
	
	// Need to check WIF is on.
//	if (TWI0.MSTATUS & TWI_WIF_bm)
//	{
		if (!(TWI0.MSTATUS & TWI_RXACK_bm) && (setup_data.byte_count < setup_data.size_byte_array))
		{
			tx_byte = *(setup_data.byte_array + setup_data.byte_count);
			//TWI0.MCTRLB |= TWI_MCMD_STOP_gc; // Strobe, always reads zero. Stop Command works...
			//TWI0.MCTRLB |= TWI_MCMD_RECVTRANS_gc; // Should wait for write to MDATA, which is the problem.
			TWI0.MDATA = tx_byte; // This register not getting written despite CLKHOLD
			// - Cannot be written during byte transmission, requires CLKHOLD bit set.
			// - Master interrupt flags cleared on MDATA write.
			// - CLKHOLD cleared by writing a 1, should be cleared automatically by MDATA write.
			setup_data.byte_count ++;
		} 
		else
		{
			TWI0.MCTRLB	|= TWI_MCMD_STOP_gc; // Stop I2C
		}
//	}
	
/*	if (TWI0.MSTATUS & TWI_RIF_bm) 
	{
		if (setup_data.byte_count < 4)
		{
			rx_byte = TWI0.MDATA;
			setup_data.byte_count ++;
		}
		else
		{
			rx_byte = TWI0.MDATA;
			TWI0.MCTRLB	|= TWI_MCMD_STOP_gc;	
		}
		
	}*/
}

ISR(TCA0_OVF_vect) {
	TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm;
	TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
	VPORTA.OUT &= ~(PIN4_bm | PIN5_bm | PIN6_bm);
}