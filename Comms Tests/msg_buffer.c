#include "msg_buffer.h"

void clear_msg_queue (struct ring_buffer *rb)
{
	rb->head = 0;
	rb->tail = 0;
	rb->status = RB_EMPTY;
	rb->overflow = 0;
}

static void increment_ring_buffer (struct ring_buffer *rb)
{
	if(rb->status < RB_FULL)
	{
		if(++rb->tail == BUFFER_SIZE) rb->tail = 0;
		rb->status = ((rb->tail) == rb->head) ? RB_FULL : RB_IN_USE;
	}
	else
	{
		rb->overflow = 1;
		rb->status = RB_ERR_OVERFLOW;
	}
}

void delete_from_msg_queue (struct ring_buffer *rb)
{
	if (rb->status != RB_EMPTY)
	{
		if(++rb->head == BUFFER_SIZE) rb->head = 0;
		rb->status = (rb->tail == rb->head) ? RB_EMPTY : RB_IN_USE;
	}
}

void add_to_msg_queue (struct ring_buffer *rb, uint8_t addr, uint8_t rw, uint8_t *data, uint8_t data_count)
{
	rb->msg_list[rb->tail].addr = addr;
	rb->msg_list[rb->tail].rw = rw;
	rb->msg_list[rb->tail].data = data;
	rb->msg_list[rb->tail].data_len = data_count;
	increment_ring_buffer(rb);
}