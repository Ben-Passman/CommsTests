#include "msg_buffer.h"

void rbuff4_clear (struct rbuff4_t *buff_ptr)
{
	buff_ptr->head = 0;
	buff_ptr->tail = 0;
	buff_ptr->status = RB_EMPTY;
	buff_ptr->overflow = 0;
}

void rbuff4_inc (struct rbuff4_t *buff_ptr)
{
	if(buff_ptr->status < RB_FULL)
	{
		buff_ptr->tail++;
		buff_ptr->status = ((buff_ptr->tail) == buff_ptr->head) ? RB_FULL : RB_IN_USE;
	}
	else
	{
		buff_ptr->overflow = 1;
		buff_ptr->status = RB_ERR_OVERFLOW;
	}
}

void rbuff4_dec (struct rbuff4_t *buff_ptr)
{
	if (buff_ptr->status != RB_EMPTY)
	{
		buff_ptr->head++;
		buff_ptr->status = (buff_ptr->tail == buff_ptr->head) ? RB_EMPTY : RB_IN_USE;
	}
}