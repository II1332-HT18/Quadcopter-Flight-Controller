
/**
******************************************************************************
* @file 
* @author: Hannes Paulsson, Ramon Rodriguez
* @version version number here
* @date    -
* @brief   Interrupt handle and task for PWM-in.
Channels on controller:
1       : Roll
2       : Thrust
3       : Pitch
4       : Yaw
5       : "Timer/ GEAR/ELE FLAP" ("Emergency break")
6       : Thrust 
trim
7       : Train CH7

Inputs, read as 0 closests to SWD
0       (probably tim8)
1       Tim4 Channel 4  CH8 PB9
2       Tim4 Channel 3  CH7 PB8
3       Tim4 Channel 2  CH6 PB7
4       Tim4 Channel 1  CH5 PB6
5       Tim3 Channel 4  CH4 PC9
6       Tim3 Channel 3  CH3 PC8
7       Tim3 Channel 2  CH2 PC7
8       Tim3 Channel 1  CH1 PC6
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Global variables ----------------------------------------------------------*/
static uint32_t thrust = 0;
static uint32_t pitch = 0;
static uint32_t roll = 0;
static uint32_t yaw = 0;
static uint32_t emergency = 0;
extern const portTickType HYPERPERIOD;
uint32_t gPwmInWDCheckback=0;

/* Function definitions -----------------------------------------------------*/
/*
* Name:  assert_failed
* Brief: write a function brief here!
* Args:  write about function arguments here!
* Rets:  write about return values here!
*/ 
void StartPwmInTask(void const * argument)
{
  portTickType  last_task_start = xTaskGetTickCount();
  while(1){
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

    
    /*Creates space on the mailbox pwmIn_mailbo*/
    pwmIn_struct *pwmIn_struct_pointer =
      (pwmIn_struct*)osMailAlloc(pwmIn_mailbox, osWaitForever);
    
    if(pwmIn_struct_pointer != NULL){

        /*
        Values are shifted to fit. Input values are between 4000 - 8000.
      Right shift by 2 to get values between 1000-2000 (what the
      pwm-out wants), and right shift by 5 and subtract 187
      to get values between -63 and +62. This is interpreted
      as degrees by the control system.
      */
      
      float pitch_value = (float) ((pitch>>5)-187.0);
      float roll_value = (float) ((roll>>5)-187.0);
      float yaw_value = (float) ((yaw>>5)-187.0);
      uint32_t thrust_value = thrust >> 2;
      uint8_t emergency_value = 0;

    
    if(emergency > 4500){
      /*
      This might be a temporary solution. As of right now,
      the quadcopter would behave the same way if it loses
      the contact with the remote controll and when the user
      hits the emergency break.
      
      This could be handled in the "reglersystem" task.
      The value "emergency" is set to 0 when everything
      is fine, 1 if the controller loses contact and 2
      if the user hits the emergency break. 
      
      */
    
        pitch_value = (float) 0.0;
        roll_value = (float) 0.0;
        yaw_value = (float) 0.0;
        thrust_value = 1000;
        emergency_value = 1;
    }
    if(emergency > 7500){
      emergency_value = 2;
    }
    
    pwmIn_struct_pointer->roll = roll_value;
    pwmIn_struct_pointer->pitch = pitch_value;
    pwmIn_struct_pointer->thrust = thrust_value;
    pwmIn_struct_pointer->yaw = yaw_value;
    pwmIn_struct_pointer->emergency = emergency_value;
    
    osMailPut(pwmIn_mailbox, pwmIn_struct_pointer);
   
    }

    gPwmInWDCheckback++;
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

    
    /*We are done ..and then we wait...*/ 
    vTaskDelayUntil(&last_task_start,HYPERPERIOD/2); 
  

  }
}

/*
* Name:  Input capture from the pwm-in.
* Args:  write about function arguments here!
* Rets:  write about return values here!
* Brief: The input comes from 3 timers, TIM3, TIM4 and TIM8.
* TIM3 and TIM4 has 4 channels.
* TIM3 has channel 1-4, and TIM4 has channel 5-8.
* TIM8 only has one channel, channel 9.
* 
* TIM 8 IS NOT IMPLEMENTED AT THE MOMENT. 
* 
* Works by first checking what TIM is causing the interrupt,
* then checking what channel. 
* Checks if the signal is HIGH (rising edge). If so, 
* it resets the counter for that channel.
* If the signal is LOW (falling edge), get the counter value
* and store that in a global variable.
*/ 
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
  
  static uint8_t rising_edge = 0;
  
  /* TIM 3 interrupt */
  if(htim->Instance == TIM3)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      rising_edge = HAL_GPIO_ReadPin(GPIOC, RC_CH1_Pin);
      if(rising_edge)
      {
        //Rising edge
        //Set timer to 0
        htim->Instance->CNT = 0;
      }
      else
      {
        //Falling edge
        //Read counter.
        roll = htim->Instance->CNT; 
      }
    }
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      rising_edge = HAL_GPIO_ReadPin(GPIOC, RC_CH2_Pin);
      if(rising_edge)
      {
        //Rising edge
        //Set timer to 0
        htim->Instance->CNT = 0;
      }
      else
      {
        //Falling edge
        //Read counter.
        thrust = htim->Instance->CNT;
      }
    }
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
      rising_edge = HAL_GPIO_ReadPin(GPIOC, RC_CH3_Pin);
      if(rising_edge)
      {
        //Rising edge
        //Set timer to 0
        htim->Instance->CNT = 0;
      }
      else
      {
        //Falling edge
        //Read counter.
        pitch = htim->Instance->CNT;
      }
    }        
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
      rising_edge = HAL_GPIO_ReadPin(GPIOC, RC_CH4_Pin);
      if(rising_edge)
      {
        //Rising edge
        //Set timer to 0
        htim->Instance->CNT = 0;
      }
      else
      {
        //Falling edge
        //Read counter.
        
        yaw = htim->Instance->CNT; 
      }
    }
  }
  /* TIM4 interrupt */
  else if(htim->Instance == TIM4)
  { 
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      rising_edge = HAL_GPIO_ReadPin(GPIOB, RC_CH5_Pin);
      if(rising_edge)
      {
        //Rising edge, set timer to 0.
        htim->Instance->CNT = 0;
      }
      else
      {
        //Falling edge, read counter.
        emergency = htim->Instance->CNT;
      }
    }
  }
}