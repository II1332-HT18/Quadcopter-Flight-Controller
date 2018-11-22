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
  
  float raw_acc_x;  //New var used for the RAW data acc_x
  float raw_acc_y;  //New var used for the RAW data acc_y
  float raw_acc_z;  //New var used for the RAW data acc_z
} FILTER_complement_struct;

/* GLOBAL FUNCTIONS ***********************************************************/
void StartsensorFilterTask(void const * argument);

#endif