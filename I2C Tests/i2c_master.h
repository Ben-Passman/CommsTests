#pragma once

#include <stdint.h>
#include <avr/io.h>

#define I2C_WRITE_bm 0x00
#define I2C_READ_bm 0x01

// BAUD = (F_CLK_PER/F_SCL - 10 - F_CLK_PER*T_RISE)/2;

void i2c_set_buffer(uint8_t *buff, uint8_t size);

void i2c_master_init();

void i2c_start (register8_t addr, register8_t read_write);