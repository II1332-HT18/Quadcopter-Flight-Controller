/**
********************************************************************************
* @file    circular_queue.c
* @author  Alexander Vassiliou
* @date    23 November 2018
* @brief   This file contains a circular implementation of the queue type
* with operations defined for enqueuing and dequeuing elements, following 
* the circular queuing policies.
********************************************************************************
*/

#include "circular_queue.h"

void circular_queue_init(AbstractQueue *queue)
{
	circular_queue_set_empty(queue);

	queue->Tail = 0;
}

bool circular_queue_enqueue(AbstractQueue *queue, void *element)
{
	if (circular_queue_is_full(queue))
		return false;

	if (circular_queue_is_empty(queue))
		queue->Head = queue->Tail;

	queue->Data[queue->Tail] = element;
	circular_queue_increment_tail(queue);

	return true;
}

bool circular_queue_dequeue(AbstractQueue *queue, void **destination)
{
	if (circular_queue_is_empty(queue))
		return false;

	*destination = queue->Data[queue->Head];
	circular_queue_increment_head(queue);

	if (queue->Head == queue->Tail)
		circular_queue_set_empty(queue);

	return true;
}

inline bool circular_queue_is_full(AbstractQueue *queue)
{
	return queue->Tail == queue->Head;
}

inline bool circular_queue_is_empty(AbstractQueue *queue)
{
	return queue->Head == -1;
}

inline void circular_queue_set_empty(AbstractQueue *queue)
{
	queue->Head = -1;
}

inline void circular_queue_increment_tail(AbstractQueue *queue)
{
	queue->Tail = (queue->Tail + 1) % queue->Size;
}

inline void circular_queue_increment_head(AbstractQueue *queue)
{
	queue->Head = (queue->Head + 1) % queue->Size;
}