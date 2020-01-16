#include "msg_buffer.h"

static struct ring_buffer rb = {.head=0, .tail=0, .status=RB_EMPTY, .overflow=0};

uint8_t get_current_address(void){	return rb.msg_list[rb.head].addr;	}

uint8_t get_current_rw_flag(void){	return rb.msg_list[rb.head].rw;	}

uint8_t *get_current_data_pointer(void){	return rb.msg_list[rb.head].data;	}

uint8_t get_current_data_length(void){	return rb.msg_list[rb.head].data_len;	}

uint8_t get_msg_queue_overflow(void){	return rb.overflow;	}

buff_state_t get_msg_queue_status(void){	return rb.status;	}

void clear_msg_overflow (void){	rb.overflow = 0;	}

void clear_msg_queue (void)
{
	rb.head = 0;
	rb.tail = 0;
	rb.status = RB_EMPTY;
	clear_msg_overflow();
}

void increment_ring_buffer (void)
{
	if(rb.status < RB_FULL)
	{
		if(++rb.tail > BUFFER_SIZE) rb.tail = BUFFER_SIZE;
		rb.status = ((rb.tail) == rb.head) ? RB_FULL : RB_IN_USE;
	}
	else
	{
		rb.overflow = 1;
		rb.status = RB_ERR_OVERFLOW;
	}
}

void delete_from_msg_queue (void)
{
	if (rb.status != RB_EMPTY)
	{
		if(++rb.head > BUFFER_SIZE) rb.head = BUFFER_SIZE;
		rb.status = (rb.tail == rb.head) ? RB_EMPTY : RB_IN_USE;
	}
}

void add_to_msg_queue (uint8_t addr, uint8_t rw, uint8_t *data, uint8_t data_count)
{
	rb.msg_list[rb.tail].addr = addr;
	rb.msg_list[rb.tail].rw = rw;
	rb.msg_list[rb.tail].data = data;
	rb.msg_list[rb.tail].data_len = data_count;
	increment_ring_buffer();
}