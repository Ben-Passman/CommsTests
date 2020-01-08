#pragma once

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define I2C_WRITE_bm 0x00
#define I2C_READ_bm 0x01

/*
	Notes:
	- TWI0 uses MADDR and MDATA registers for data transfer. MADDR contains a 7-bit address and the R/W bit, MDATA contains 8 bits of data.
*/

// BAUD = (F_CLK_PER/F_SCL - 10 - F_CLK_PER*T_RISE)/2;
#define BAUD 0x0B

void i2c_master_init();
void i2c_start (register8_t addr, register8_t read_write, uint8_t *data, uint8_t data_size);