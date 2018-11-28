/**
********************************************************************************
* @file    moving_average.h
* @author  Alexander Vassiliou
* @date    23 November 2018
* @brief   Header file for moving_average.c
********************************************************************************
*/

#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include "abstract_queue.h"

/**
* @brief Data type used to represent the average of a moving subset of data points, 
* continuously shifting forward in time.
*
* @param Sum	 Represents the last calculated sum of the current subset of data points.
* @param Average Represents the last calculated average of the current subset of data points.
* @param Data	 Represents the pointer to the current subset of data points to be averaged.
**/
typedef struct
{
	void *Sum;
	void *Average;

	AbstractQueue *Data;
} MovingAverage;

/**
* @brief Dynamically allocate a new moving average of a given size.
*
* @param size Represents the maximum number of data points.
*
* @return A pointer to the newly allocated moving average.
**/
MovingAverage *moving_average_allocate(int size);

/**
* @brief Deallocate a dynamically allocated moving average.
*
* @param maverage A pointer to the moving average.
*
* @return void
**/
void moving_average_deallocate(MovingAverage *maverage);

/**
* @brief Initialize a moving average to zero.
* 
* @param maverage A pointer to the moving average.
* 
* @return void
**/
void moving_average_init(MovingAverage *maverage);

/**
* @brief Move the average by removing the oldest data point and adding the 
* new data point to the subset.
* 
* @param maverage A pointer to the moving average.
* @param newPoint Representing the new data point to replace the oldest one.
* 
* @return void
**/
void moving_average_move(MovingAverage *maverage, float newPoint);

/**
* @berief Get the last calculated sum as a floating point value.
*
* @param maverage A pointer to the moving average.
*
* @return A floating point value of the last calculated sum.
**/
inline float moving_average_get_sum_float(MovingAverage *maverage);

/**
* @berief Get the last calculated average as a floating point value.
* 
* @param maverage A pointer to the moving average.
* 
* @return A floating point value of the last calculated average.
**/
float moving_average_get_average_float(MovingAverage *maverage);

#endif