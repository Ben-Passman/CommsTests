#pragma once

#include <stdint.h>
#include <avr/io.h>

void spi_set_buffers(const uint8_t *tx_buff, uint8_t *rx_buff, uint8_t size);

void spi_master_init();

void spi_start ();