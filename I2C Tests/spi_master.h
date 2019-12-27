#pragma once

#include <stdint.h>
#include <avr/io.h>

typedef void (*cb) (void);

void spi_master_init();

void spi_start (const uint8_t *tx_buff, uint8_t *rx_buff, uint8_t size, cb callback);