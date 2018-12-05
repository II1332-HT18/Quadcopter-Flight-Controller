
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

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern const portTickType HYPERPERIOD;
//For test purposes
uint32_t gAnalysWDCheckback=0;


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
  // Get current time
  portTickType last_task_start = xTaskGetTickCount();
  
  // Fetch data from mailboxes
  osEvent check_mail = osMailGet(sensorFilter_mailbox, osWaitForever);
  FILTER_complement_struct *filter_data = (FILTER_complement_struct*)check_mail.value.p;
  
  osEvent check_mail2 = osMailGet(analys_mailbox, osWaitForever);
  main_struct *control_data = (main_struct*)check_mail2.value.p;
  
  
  //***************BLUETOOTH******************
  //A 10 sec delay on the thread to have time to connect to Bluetooth. Otherwise not necessary.
  //vTaskDelay(10000);
  //******************************************
  
  /* Thread */
  while(1){
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

    float val[11];
    uint8_t ctr = 0;
    
    val[ctr++] = 1337; //PREAMBLE SHOULD ALWAYS BE SENT
    /*
     *CHOOSE 10 VAlUES TO BE SENT AS CHANNELS TO SERIALPLOT
     *THEY WILL BE IN CHRONOLOGICAL ORDER, E.G. CHANNEL #1 WILL BE SENT FIRST
     */

    //Filter data
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
//    val[ctr++] = filter_data->filter_yaw;
//    
//    
//    //Control data
//    val[ctr++] = control_data->PIDoutputGyroYaw.f;       //TODO:Change name of errorgyroyaw
//    val[ctr++] = control_data->PIDoutputPitch.f;
//    val[ctr++] = control_data->PIDoutputRoll.f;
//    
//   
//    //Motors
//    val[ctr++] = control_data->RFmotor.f;
//    val[ctr++] = control_data->LFmotor.f;
//    val[ctr++] = control_data->RBmotor.f;
//    val[ctr++] = control_data->LBmotor.f;
//    
//    
//    //Remote
//    val[ctr++] = control_data->yaw.f;
//    val[ctr++] = control_data->pitch.f;
//    val[ctr++] = control_data->roll.f;
//    val[ctr++] = control_data->thrust;
//    val[ctr++] = control_data->emergency;
     
    /*
    for(int i=0; i<ctr; i++){
    
    }
    */
    // Transmit signals to UART peripheral
    HAL_UART_Transmit(&huart3,(uint8_t*)&val, sizeof(float)*ctr, 4);
    huart3.State = HAL_UART_STATE_READY;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    // Sleep thread until end of hyperperiod
    vTaskDelayUntil(&last_task_start,HYPERPERIOD); 
  } 
}