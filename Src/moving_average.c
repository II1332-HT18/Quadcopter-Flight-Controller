/**
********************************************************************************
* @file    moving_average.c
* @author  Alexander Vassiliou
* @date    23 November 2018
* @brief   This file contains a moving average filter implementation, with
* operations to move the average by adding a new data point and reading the 
* last calculated sum and average.
********************************************************************************
*/

#include "moving_average.h"

#include "circular_queue.h"

#include "stdlib.h"

MovingAverage *moving_average_allocate(int size)
{
	MovingAverage *maverage;
	AbstractQueue *queue;
	
	queue = abstract_queue_allocate(size);
	circular_queue_init(queue);

	maverage = (MovingAverage *)malloc(sizeof(MovingAverage));
	maverage->Data = queue;

	return maverage;
}

void moving_average_deallocate(MovingAverage *maverage)
{
	abstract_queue_deallocate(maverage->Data);

	free(maverage);
}

void moving_average_init(MovingAverage *maverage)
{
	int i;

	for (i = 0; i < maverage->Data->Size; i++)
		circular_queue_enqueue(maverage->Data, 0);

	maverage->Sum = 0;
	maverage->Average = 0;
}

void moving_average_move(MovingAverage *maverage, float newPoint)
{
	float oldPoint;
	float average;
	float sum;

	circular_queue_dequeue(maverage->Data, (void **)&oldPoint);
	circular_queue_enqueue(maverage->Data, *(void **)&newPoint);

	sum = moving_average_get_sum_float(maverage) - oldPoint + newPoint;
	average = sum / maverage->Data->Size;

	maverage->Sum = *(void **)&sum;
	maverage->Average = *(void **)&average;
}

inline float moving_average_get_sum_float(MovingAverage *maverage)
{
	return *(float *)&maverage->Sum;
}

float moving_average_get_average_float(MovingAverage *maverage)
{
	return *(float *)&maverage->Average;
}