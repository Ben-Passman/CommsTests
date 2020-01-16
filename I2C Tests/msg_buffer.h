#pragma once

#include <stdio.h>

#define BUFFER_SIZE 4

struct message
{
	uint8_t addr : 7;
	uint8_t rw : 1;
	uint8_t *data;
	uint8_t data_len;
};

struct ring_buffer
{
	uint8_t head;
	uint8_t tail;
	uint8_t status : 2;
	uint8_t overflow : 1;
	struct message msg_list[BUFFER_SIZE];
};

typedef enum buff_state {
	RB_EMPTY=0,
	RB_IN_USE,
	RB_FULL,
	RB_ERR_OVERFLOW
} buff_state_t;

uint8_t get_current_address(void);
uint8_t get_current_rw_flag(void);
uint8_t *get_current_data_pointer(void);
uint8_t get_current_data_length(void);
uint8_t get_msg_queue_overflow(void);
buff_state_t get_msg_queue_status(void);
void clear_msg_overflow (void);
void clear_msg_queue (void);
void add_to_msg_queue (uint8_t addr, uint8_t rw, uint8_t *data, uint8_t data_count);
void delete_from_msg_queue (void);

#define Q_ADDR get_current_address()
#define Q_RW get_current_rw_flag()
#define Q_DATA get_current_data_pointer()
#define Q_DATA_LEN get_current_data_length()