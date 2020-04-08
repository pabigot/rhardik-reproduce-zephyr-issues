
#include <fieldsightlib.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#include "circular_buffer.h"

static void advance_pointer(cbuf_handle_t cbuf)
{
	assert(cbuf);

	if(cbuf->full)
    {
        cbuf->tail = (cbuf->tail + 1) % cbuf->max;
    }

	cbuf->head = (cbuf->head + 1) % cbuf->max;

	// We mark full because we will advance tail on the next time around
	cbuf->full = (cbuf->head == cbuf->tail);
}

static void retreat_pointer(cbuf_handle_t cbuf)
{
	assert(cbuf);

	cbuf->full = false;
	cbuf->tail = (cbuf->tail + 1) % cbuf->max;
}

cbuf_handle_t circular_buf_init(struct circular_buf_t *cbuf, uint8_t* buffer, size_t size)
{
	assert(buffer && size);
	assert(cbuf);

	cbuf->buffer = buffer;
	cbuf->max = size;
	circular_buf_reset(cbuf);

	assert(circular_buf_empty(cbuf));

	return cbuf;
}

#if 0
void circular_buf_free(cbuf_handle_t cbuf)
{
	assert(cbuf);
	free(cbuf);
}
#endif
void circular_buf_reset(cbuf_handle_t cbuf)
{
    assert(cbuf);

    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;
}

size_t circular_buf_size(cbuf_handle_t cbuf)
{
	assert(cbuf);

	size_t size = cbuf->max;

	if(!cbuf->full)
	{
		if(cbuf->head >= cbuf->tail)
		{
			size = (cbuf->head - cbuf->tail);
		}
		else
		{
			size = (cbuf->max + cbuf->head - cbuf->tail);
		}

	}

	return size;
}
size_t circular_buf_avail_bytes(cbuf_handle_t cbuf)
{
	return cbuf->max - circular_buf_size(cbuf);
}
size_t circular_buf_capacity(cbuf_handle_t cbuf)
{
	assert(cbuf);

	return cbuf->max;
}

K_SEM_DEFINE(cbuf_put_sem,1,1);
int circular_buf_put(cbuf_handle_t cbuf, uint8_t *data, uint8_t len)
{
	uint16_t i;
	size_t avail = circular_buf_avail_bytes(cbuf);

		if(avail < len)
			return -1;
	k_sem_take(&cbuf_put_sem, K_FOREVER);	
	for (i = 0; i < len; ++i) {
		cbuf->buffer[cbuf->head] = data[i];
		advance_pointer(cbuf);
	}
	k_sem_give(&cbuf_put_sem);
	return 0;
}

K_SEM_DEFINE(cbuf_get_sem, 1, 1);
int circular_buf_get(cbuf_handle_t cbuf, uint8_t * data)
{
	int r = -1;

	k_sem_take(&cbuf_get_sem, K_FOREVER);	
	if(!circular_buf_empty(cbuf))
	{
		*data = cbuf->buffer[cbuf->tail];
		retreat_pointer(cbuf);

		r = 0;
	}
	k_sem_give(&cbuf_get_sem);

	return r;
}

bool circular_buf_empty(cbuf_handle_t cbuf)
{
	assert(cbuf);

    return (!cbuf->full && (cbuf->head == cbuf->tail));
}

bool circular_buf_full(cbuf_handle_t cbuf)
{
	assert(cbuf);

    return cbuf->full;
}
