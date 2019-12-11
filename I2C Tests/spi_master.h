#pragma once

#include <stdint.h>
#include <avr/io.h>

// MOSI	PA1/PC2
// MISO	PA2/PC1
// SCK	PA3/PC0
// SS	PA4/PC3

void spi_set_buffer(uint8_t *buff, uint8_t size);

void spi_master_init();

void spi_start ();