/**
********************************************************************************
* @file    abstract_queue.c
* @author  Alexander Vassiliou
* @date    26 November 2018
* @brief   This file contains an implementation of a general abstract queue type 
* with no operations.
********************************************************************************
*/

#include "abstract_queue.h"

#include "stdlib.h"

AbstractQueue *abstract_queue_allocate(int size)
{
	const int sizeof_pointer = sizeof(void*);

	AbstractQueue *queue;
	void **data;

	queue = (AbstractQueue *)malloc(sizeof(AbstractQueue));
	data = malloc(sizeof_pointer * size);

	queue->Size = size;
	queue->Data = data;

	return queue;
}

void abstract_queue_deallocate(AbstractQueue *queue)
{
	free(queue->Data);
	free(queue);
}