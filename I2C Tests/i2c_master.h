#pragma once

#include <stdint.h>
#include <avr/io.h>

#define I2C_WRITE_bm 0x00
#define I2C_READ_bm 0x01

/*
	Notes:
	- TWI0 uses MADDR and MDATA registers for data transfer. MADDR contains a 7-bit address and the R/W bit, MDATA contains 8 bits of data.
*/

// BAUD = (F_CLK_PER/F_SCL - 10 - F_CLK_PER*T_RISE)/2;

void i2c_set_buffer(uint8_t *buff, uint8_t size);

void i2c_master_init();

void i2c_start (register8_t addr, register8_t read_write);