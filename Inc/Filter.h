/*******************************filter.h***************************************/
#ifndef __FILTER_H
#define __FILTER_H

/* GLOBAL DEFINES *************************************************************/

/* Mailboxes */
extern osMailQId sensorFilter_mailbox;

/* STRUCTS ********************************************************************/
typedef struct {
  float val1[5];
  float val2[5];
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
  
  // 2018-11-2X added space to send RAW acc data
  float raw_acc_x;  //New var used for the RAW data acc_x
  float raw_acc_y;  //New var used for the RAW data acc_y
  float raw_acc_z;  //New var used for the RAW data acc_z
  
  // 2018-12-06 added space for 'change speed' since the control-group needs it
  float pitch_angle_speed;    // New var used for speed of change, the derivative
  float roll_angle_speed;     // New var used for speed of change, the derivative
  float yaw_angle_speed;      // New var used for speed of change, the derivative
  
} FILTER_complement_struct;

// Struct to store old and current value in order to calculate angel speed
typedef struct {
  float old;            // Holds old value
//  float current;        // Holds the current value
} FILTER_angle_speed_struct;
  

/* GLOBAL FUNCTIONS ***********************************************************/
void StartsensorFilterTask(void const * argument);

#endif