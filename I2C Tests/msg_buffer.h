#pragma once

#include <stdio.h>

enum rbuff_state {
	RB_EMPTY=0,
	RB_IN_USE,
	RB_FULL,
	RB_ERR_OVERFLOW
};

struct rbuff4_t
{
	uint8_t head:2;
	uint8_t tail:2;
	uint8_t status:2;
	uint8_t overflow:1;
};

void rbuff4_clear (struct rbuff4_t *buff_ptr);
void rbuff4_inc (struct rbuff4_t *buff_ptr);
void rbuff4_dec (struct rbuff4_t *buff_ptr);