
/** ****************************************************************************
* @file
* @author  Simon Strom, Henrik Bjorklund
* @version 1.0
* @date    2017-11-24
* @author  Johan Norberg & Carl Uddman 
* @version 0.1
* @date    2016-12-05
*
* @brief Analysis functionality
*
* @detail Functions for use with serial plot program to analyze the contents of
* various signals in the flight controller system. Use the boolean selection
* variables to pick up to 5 signals to output.
*
* @todo Rewrite to use new structs.
* @todo Receive data to be plotted through the mailbox.
*
* @bug Serial output is currently hardcoded where needed.
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "analys.h"
#include "accelerometer_lis3dh.h"
#include "gyroscope_l3gd20h.h"
#include <stdio.h>

//#define MAX_ANALYSE_CHANNELS 4

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern const portTickType MAIN_FREQUENCY;
//For test purposes
uint32_t gAnalysWDCheckback=0;

//static int16_t val[MAX_ANALYSE_CHANNELS];

/* Thread definitions --------------------------------------------------------*/
/** ****************************************************************************
* @brief Analysis task
* 
* @detail Thread that outputs various system signals using UART for analysis 
* with serial communication plot. Use the private variables defined 
* above to select which 5 signals to output by setting them to TRUE.
*******************************************************************************/
void StartAnalysTask(void const * argument)
{ 
  portTickType last_task_start = xTaskGetTickCount();
  
  osEvent check_mail = osMailGet(sensorFilter_mailbox, osWaitForever);
  FILTER_complement_struct *filter_data = (FILTER_complement_struct*)check_mail.value.p;
  
//    ACC_TypeDef acc_values;
//    GYR_TypeDef gyr_values;
  
  /* Thread */
  while(1)
  {
    
//    ACC_update_xyz();
//    GYR_update_xyz();
//    
//    val[0] = acc_values.x_raw;
//    val[1] = acc_values.y_raw;
//    val[2] = acc_values.z_raw;
//    val[3] = gyr_values.x_raw;
//    val[4] = gyr_values.y_raw;
//    val[5] = gyr_values.z_raw;
//    val[6] = xTaskGetTickCount();
    
    
    // Send data with UART.
//    float val[11];
//    uint8_t ctr = 0;
//    val[ctr++] = xTaskGetTickCount();
//    val[ctr++] = lowpass_data_acc_x.output;
//    val[ctr++] = lowpass_data_acc_y.output;
//    val[ctr++] = lowpass_data_acc_z.output; 
//    val[ctr++] = acc_raw.x_raw;
//    val[ctr++] = acc_raw.z_raw;
//    val[ctr++] = acc_raw.y_raw;
//    val[ctr++] = complement_data.filter_pitch;
//    val[ctr++] = complement_data.filter_roll;
//    val[ctr++] = complement_data.acc_pitch;
//    val[ctr++] = complement_data.acc_roll;
    
    //might have to use "->"
    //do this with loop?
    float val[11];
    uint8_t ctr = 0;
    //val[ctr++] = xTaskGetTickCount();
    val[ctr++] = filter_data->acc_x;
    val[ctr++] = filter_data->acc_y;
    val[ctr++] = filter_data->acc_z;
    val[ctr++] = filter_data->gyr_x;
    val[ctr++] = filter_data->gyr_y;
    val[ctr++] = filter_data->gyr_z;
    val[ctr++] = filter_data->acc_pitch;
    val[ctr++] = filter_data->acc_roll;
    val[ctr++] = filter_data->filter_pitch;
    val[ctr++] = filter_data->filter_roll;
    val[ctr++] = filter_data->filter_yaw;
    
      
    // Transmit signals to UART peripheral
    HAL_UART_Transmit(&huart3,(uint8_t*)&val, sizeof(float)*ctr, 4);
    huart3.State = HAL_UART_STATE_READY;
    
    
    // Sleep thread until end of hyperperiod
    vTaskDelayUntil(&last_task_start,MAIN_FREQUENCY); 
  } 
}