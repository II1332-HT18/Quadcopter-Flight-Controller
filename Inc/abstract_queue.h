/**
********************************************************************************
* @file    abstract_queue.h
* @author  Alexander Vassiliou
* @date    26 November 2018
* @brief   Header file for abstract_queue.c
********************************************************************************
*/

#ifndef ABSTRACT_QUEUE_H
#define ABSTRACT_QUEUE_H

/**
* @brief Data type used to represent a queue of elements, using the FIFO policy.
*
* @param Size Represents the size of the queue.
* @param Head Represents the index of the next dequeued element.
* @param Tail Represents the index of the next enqueued element.
* @param Data Represents the pointer to the elements in the queue.
**/
typedef struct
{
	int	Size;

	int Head;
	int	Tail;

	void **Data;
} AbstractQueue;

/**
* @brief Dynamically allocate a new queue of a given size.
*
* @param size Represents the maximum number of elements.
*
* @return A pointer to the newly allocated queue.
**/
AbstractQueue *abstract_queue_allocate(int size);

/**
* @brief Deallocate a dynamically allocated queue.
*
* @param queue A pointer to the queue.
*
* @return void
**/
void abstract_queue_deallocate(AbstractQueue *queue);

#endif