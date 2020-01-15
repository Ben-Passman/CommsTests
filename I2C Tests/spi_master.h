#pragma once

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

typedef enum SPI_status_enum
{
	SPI_BUSY = 0,
	SPI_RX_BUFF_FULL,
	SPI_TX_BUFF_ERR,
	SPI_RX_BUFF_ERR,
	SPI_COL_ERR,
	SPI_IDLE
} spi_status_t;

spi_status_t spi_master_init();
spi_status_t spi_start (const uint8_t *tx_buff, uint8_t *rx_buff, uint8_t byte_count);
spi_status_t get_spi_status (void);