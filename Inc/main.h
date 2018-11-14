/**
********************************************************************************
* @file    main.h
* @author  author names here
* @version version number here
* @date    most recent date here
* @brief   header file for main.c
********************************************************************************
*/

#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "pindef.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>

typedef union _data {
  float f;
  uint8_t  i[4];
} float2int;

/* Defenitions of structs used in mailbox communication ----------------------*/
/* Main struct */
typedef struct {
  /* Sensor values*/
  float2int accel_pitch_angle;
  float2int accel_roll_angle;
  
  float2int gyro_pitch_rate;
  float2int gyro_roll_rate;
  float2int gyro_yaw_rate;
  float2int previous_gyro_yaw_rate;
  
  /* Filtered values*/
  float2int filtered_pitch_angle;
  float2int filtered_roll_angle;
  float2int filtered_yaw_angle;
  
  /* Low-pass filtered accelerometer values*/
  
  float2int acc_filtered_pitch;
  float2int acc_filtered_roll;
  
  /* RC input */
  float2int     roll;
  float2int     pitch;
  float2int     yaw;
  uint32_t      thrust; 
  uint8_t       emergency;
  
  /* Errors from PID-regulator */ 
  /* yaw */
  float2int     errorGyroYaw;          
  /* pitch */
  float2int     errorPitch;
  float2int     errorGyroPitch;
  /* roll */
  float2int     errorRoll;
  float2int     errorGyroRoll;
   
  /*Thrust values*/
  float2int     RFmotor;  
  float2int     LFmotor;
  float2int     RBmotor;
  float2int     LBmotor;
  
  /* Control-signals from PID-regulator */
  /* yaw */
  float2int     PIDoutputGyroYaw;
  /* pitch */
  float2int     PIDoutputPitch;
  float2int     PIDoutputGyroPitch;
  /* roll */
  float2int     PIDoutputRoll;
  float2int     PIDoutputGyroRoll;
  
  
  /* Time derivatives (dt) */
  float2int     dtControll;
  float2int     dtAnalys;
  
  float         x_acc;
  float         y_acc;
  float         z_acc;
  
} main_struct;

/* PWM In struct */
typedef struct {
  float         roll;
  float         pitch;
  float         yaw;
  uint32_t      thrust; 
  uint8_t       emergency;
} pwmIn_struct;

/*---------------- User includes Threads.h ------------*/
#include "pwmIn.h"
#include "controlThread.h"
/*---------------- User include Functions.h ----------*/
#include "systemInit.h"
#include "analys.h"
#include "sensor.h"
#include "filter.h"
#include "automaticControl.h"


//watchdog specific code

typedef struct _threadArgs{
  uint16_t              led;
  uint8_t               id;
  uint32_t              runningTime;
}threadArgs;


typedef struct _genericArg{
  char*                 type;
  void**                 arg;
}genericArg;

/*bruteforce every-fucking-thing
or this is defined as a goddamn const in cmsis_os.h
So i redefine it for reasons
*/
typedef struct _os_thread_def_UD{
  char*                 name;        /* Thread name                                               */
  os_pthread            pthread;      /* start address of thread function                          */
  osPriority            tpriority;    /* initial thread priority                                   */
  uint32_t              instances;    /* maximum number of instances of that thread function       */
  uint32_t              stacksize;    /* stack size requirements in bytes; 0 is default stack size */
}threadDef;

//typedef _os_thread_def_UD threadDef;

typedef struct _threadInfo{
  uint32_t size;
  osThreadId*           ID;
  threadDef*            tDef;
  void**                 arg;
}threadInfo;

void* copy_osThreadDef_UD_t(threadDef* destination, osThreadDef_t* source);
void loadHandleThread(void const* arguments);

#endif /* __MAIN_H */