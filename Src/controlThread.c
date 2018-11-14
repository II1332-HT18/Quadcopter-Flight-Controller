/**
******************************************************************************
* @file    controlThread.c
* @author  write authors here
* @date    5 November 2016
* @brief   Contains RTOS thread for control system
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_OC_InitTypeDef sConfigOC;
//extern unsigned long start;
extern uint8_t kalman;

/* Private variables ---------------------------------------------------------*/
/* Watchdog check back */
static uint32_t gMainloopWDCheckback = 0;
/* Pointers for running avarage filter */
/* Never referenced */
//!debug
//TODO check if this stuff is still needed. remove if it isn't
//static float   rollBuffer[BUFFERSIZE];
//static float   pitchBuffer[BUFFERSIZE];
//static float*  rollStartPointer = rollBuffer;
//static float*  pitchStartPointer = pitchBuffer;
//static float past_filter_value = 0.0;
//static float present_filter_value = 1.0;
//static int count = 0;
//static float*  rollEndPointer = (rollBuffer + BUFFERSIZE-1);
//static float*  rollBufferPointer;
//static float*  pitchEndPointer = (pitchBuffer + BUFFERSIZE-1); 
//static float*  pitchBufferPointer; 

/* Function definitions ------------------------------------------------------*/
/*
* Name:  StartControlTask
* Brief: Runs control system
* Args:  freeRTOS argument
* Rets:  null
*/
void StartControlTask(void const * arguments)
{
  /* Set up */
  bool warmStart = (bool)arguments;
  //start = xTaskGetTickCount();

  /* Declare local variables for the task */
  const portTickType CONTROLL_FREQUENCY = 2;  
  portTickType  last_task_start = xTaskGetTickCount();
  main_struct *all_values = pvPortMalloc(sizeof(main_struct));
  
  /* Set up pointers */
  //!debug
  //TODO check if this stuff is still needed. remove if it isn't
//  rollBufferPointer = rollStartPointer;
//  pitchBufferPointer = pitchStartPointer;

  /* Start motors and handle cold start */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  if(!warmStart)osDelay(5000);
  
  /* Main loop */
  while(1)
  {
    /* Get RC control PWM values */
    getPWMinValues(all_values);
    osMailPut(sensorFilter_mailbox, all_values);
    
    /* Check filtered values */
    osEvent check_mail = osMailGet(sensorFilter_mailbox, osWaitForever);
    all_values = check_mail.value.p; 
  
    //!debug
    //TODO check if this stuff is still needed. remove if it isn't
//    present_filter_value = all_values -> filtered_roll_angle.f;
//    if(present_filter_value == past_filter_value)
//    {
//      count++;
//    }
//    past_filter_value = all_values -> filtered_roll_angle.f;
    
    /* Run control system */
    automaticControl(all_values);
    
    /* Watchdog check back */
    gMainloopWDCheckback++;

    /* Delay the task to a fixed time to ensure constant execution frequncy */
    vTaskDelayUntil(&last_task_start,CONTROLL_FREQUENCY); 
  }
}

/*
* Name:  getPWMinValues
* Brief: extracts radio control PWM values from mailbox
* Args:  pointer to global value struct
* Rets:  null
*/
void getPWMinValues(main_struct* all_values)
{  
  /* Set up */
  pwmIn_struct* message;
  
  /* Checks for mail */
  osEvent  evt;
  evt = osMailGet(pwmIn_mailbox, 0); 

  /* If PWMin have sent mail, save data to all_values */ 
  if (evt.status == osEventMail) 
  {          
    message = evt.value.p;
    all_values->pitch.f = message->pitch;
    all_values->roll.f = message->roll;
    all_values->yaw.f = message->yaw;
    all_values->thrust = message->thrust;
    all_values->emergency = message->emergency;
    /* Free mailbox */
    osMailFree(pwmIn_mailbox, message);     
  }
  
  /* Return */
  return;
}