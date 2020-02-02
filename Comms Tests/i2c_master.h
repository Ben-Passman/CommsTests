#pragma once

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define I2C_WRITE_bm 0x00
#define I2C_READ_bm 0x01

typedef enum I2C_events_enum
{
	tx_complete = 0,
	rx_complete,
	address_NACK_error,
	data_NACK_error,
	ARB_error,
	bus_error
} i2c_events_t;

typedef enum i2c_ops_enum
{
	restart_i2c,
	stop_i2c,
	reset_i2c
} i2c_operations_t;

typedef i2c_operations_t(*i2c_event_callback)(void);

/*
	Notes:
	- TWI0 uses MADDR and MDATA registers for data transfer. MADDR contains a 7-bit address and the R/W bit, MDATA contains 8 bits of data.
*/

void i2c_master_init();
void i2c_set_stop();
void i2c_set_restart();
void i2c_set_buffer(uint8_t slave_addr, uint8_t *data, uint8_t byte_count);
void i2c_set_event_callback(i2c_events_t ev, i2c_event_callback cb);
void i2c_start ();
uint8_t i2c_idle();