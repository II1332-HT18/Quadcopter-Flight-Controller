/*******************************filter.h***************************************/

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
  float acc_x;
  float acc_y;
  float acc_z;
  
  float gyr_x;
  float gyr_y;
  float gyr_z;
  
  float acc_pitch;
  float acc_roll;
  
  float filter_pitch;
  float filter_roll;
  float filter_yaw;
} FILTER_complement_struct;

/* GLOBAL FUNCTIONS ***********************************************************/
void StartsensorFilterTask(void const * argument);

#endif