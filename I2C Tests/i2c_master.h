#pragma once

#include <stdint.h>
#include <avr/io.h>

#define I2C_WRITE 0x00
#define I2C_READ 0x01


typedef enum I2C_states_enum
{
	I2C_BUSY = 0,
	I2C_IDLE,
	I2C_ERR,
	I2C_START,
	I2C_TX_BYTE,
	I2C_RX_BYTE
} i2c_states_t;

typedef struct
{
	uint8_t *byte_array;
	uint8_t size_byte_array;
	uint8_t byte_count;
} I2C_data;

// BAUD = (F_CLK_PER/F_SCL - 10 - F_CLK_PER*T_RISE)/2;

void i2c_master_init();

i2c_states_t i2c_start (register8_t addr, register8_t read_write);

i2c_states_t i2c_write (register8_t data);

i2c_states_t i2c_stop();