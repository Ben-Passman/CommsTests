#pragma once

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

typedef enum spi_commands_enum 
{
	restart_spi = 0,
	stop_spi,
	reset_spi
} spi_command_t;

typedef spi_command_t (*spi_event_callback)(void);

void spi_master_init();
void spi_start (const uint8_t *tx_buff, uint8_t *rx_buff, uint8_t byte_count);
void set_transmit_complete_callback(spi_event_callback cb);
void set_data_error_callback(spi_event_callback cb);
uint8_t spi_idle (void);