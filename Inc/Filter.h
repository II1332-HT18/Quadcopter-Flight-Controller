/*******************************************************************************
* @file    filter.c
*
* @author  Alpha Fofana, Carl Mossberg, Simon Lagerqvist, Tommie H?glund Gran
* @version 1.2
* @date    2018-11-22
*******************************filter.h***************************************/
#ifndef __FILTER_H
#define __FILTER_H

/* GLOBAL DEFINES *************************************************************/

/* Mailboxes */
extern osMailQId sensorFilter_mailbox;

/* STRUCTS ********************************************************************/
typedef struct {
  float val1[3];
  float val2[3];
  float output; // Filtered output value
} FILTER_lowpass_struct;

typedef struct {
  float acc_x;  //Used for lowpass filtered acc_x
  float acc_y;  //Used for lowpass filtered acc_y
  float acc_z;  //Used for lowpass filtered acc_z

  float gyr_x;  //Used for raw gyro_x
  float gyr_y;  //Used for raw gyro_y
  float gyr_z;  //Used for raw gyro_z

  float acc_pitch;  //Used for the calculated (arctan2) acc_pitch
  float acc_roll;   //Used for the calculated (arctan2) acc_roll

  float filter_pitch;  //Used for complement filtered pitch
  float filter_roll;   //Used for complement filtered roll
  float filter_yaw;    //Used for complement filtered yaw

} FILTER_complement_struct;

// Struct to store old and current value in order to calculate angel speed
typedef struct {
  float old;            // Holds old value
  float older;        // Holds older value in order to calc roling mean value
  float oldest;         //Holds oldest value
} FILTER_angle_speed_struct;

/* GLOBAL FUNCTIONS ***********************************************************/
void StartsensorFilterTask(void const * argument);

#endif
