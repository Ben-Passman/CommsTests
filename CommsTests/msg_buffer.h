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

#define Q_ADDR(b) b.msg_list[b.head].addr
#define Q_RW(b) b.msg_list[b.head].rw
#define Q_DATA(b) b.msg_list[b.head].data
#define Q_DATA_LEN(b) b.msg_list[b.head].data_len

void clear_msg_queue (struct ring_buffer *rb);
void add_to_msg_queue (struct ring_buffer *rb, uint8_t addr, uint8_t rw, uint8_t *data, uint8_t data_count);
void delete_from_msg_queue (struct ring_buffer *rb);