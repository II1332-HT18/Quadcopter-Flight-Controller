/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main quadrocopter program
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright 
  *      notice, this list of conditions and the following disclaimer in the 
  *      documentation and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contribu-
  *      tor may be used to endorse or promote products derived from this 
  *      software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSE-
  * QUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
  * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
  * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
  * DAMAGE.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Thread IDs ----------------------------------------------------------------*/
osThreadId controlTaskHandle;
osThreadId pwmInTaskHandle;
osThreadId analysTaskHandle;
osThreadId loadTaskHandle;
osThreadId sensorFilterTaskHandle;

/* Mailbox Definitions--------------------------------------------------------*/
/* PWM In */
osMailQDef(pwmIn_mailbox, 1, pwmIn_struct);
osMailQId pwmIn_mailbox;

/* Analysis */
osMailQDef(analys_mailbox, 1, main_struct);
osMailQId analys_mailbox;

/* sensorFilter */
osMailQDef(sensorFilter_mailbox, 1, main_struct);
osMailQId sensorFilter_mailbox;

/* Global setting variables --------------------------------------------------*/
/* Global, iNemo or the other one. 1 for iNemo, 0 for the other one. */
uint8_t iNemo = 0; 
/* Global. Kalman or complementary. 1 for kalman, 0 for complementary */
uint8_t kalman = 0;
/* Global frequency */
const portTickType MAIN_FREQUENCY = 4; //Q: what is this used for? SHOULD BEE 4

/* Function definitions ------------------------------------------------------*/
/* Main program body */
int main(void)
{
  /* Calls the function that initializes the code generated from cubeMX*/
  systemInit();
  
  
  /* Test for cold or warm start .kollar på watch dog */
  RCC_TypeDef rcc;
  
  bool warmStart = false;
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    warmStart=true;
  }
  else
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();      
  }
  
  /* Create the threads ------------------------------------------------------*/
  /* Control thread */
  osThreadDef(controlTask, StartControlTask, osPriorityNormal, 1, 128);
  controlTaskHandle = osThreadCreate(osThread(controlTask), (void*)warmStart);
  
  /* PWM-input thread */
  osThreadDef(pwmInTask, StartPwmInTask, osPriorityNormal, 1, 128);
  pwmInTaskHandle = osThreadCreate(osThread(pwmInTask), NULL);
  
  /* Analysis thread */
  osThreadDef(analysTask, StartAnalysTask, osPriorityLow, 1, 128);
  analysTaskHandle = osThreadCreate(osThread(analysTask), NULL);
  
  /* Sensor and filtering thread */
  osThreadDef(sensorFilterTask, StartsensorFilterTask, 
    osPriorityAboveNormal, 1, 128);
  sensorFilterTaskHandle = osThreadCreate(osThread(sensorFilterTask), NULL);
  
  /* Setup start argument for loadHandler - sysmonitor task */
  threadInfo* tInfo     = pvPortMalloc(sizeof(threadInfo));
  tInfo->size           = 3;
  tInfo->ID             = pvPortMalloc(sizeof(osThreadId)*3);
  tInfo->tDef           = pvPortMalloc(sizeof(threadDef)*3); 
  tInfo->arg            = (void*)pvPortMalloc(sizeof(void*)*3);
  
  /* Save thread handles. Take not that the order is the same  as the order 
   they ack the loadHandler thread through gCheckback */
  tInfo->ID[0] = controlTaskHandle;
  tInfo->ID[1] = pwmInTaskHandle;
  tInfo->ID[2] = analysTaskHandle;
 
  /* Copy thread data, note the order */
  copy_osThreadDef_UD_t(&(tInfo->tDef[0]),(struct os_thread_def const*)
    &(os_thread_def_controlTask));
  copy_osThreadDef_UD_t(&(tInfo->tDef[1]),(struct os_thread_def const*)
    &(os_thread_def_pwmInTask));
  copy_osThreadDef_UD_t(&(tInfo->tDef[2]),(struct os_thread_def const*)
    &(os_thread_def_analysTask));
  
  /* Copy thread arguments */
  tInfo->arg[0] = NULL;
  tInfo->arg[1] = NULL;
  tInfo->arg[2] = NULL;
 
  /* Create mailboxes */  
  sensorFilter_mailbox = osMailCreate(osMailQ(sensorFilter_mailbox), NULL);
  pwmIn_mailbox = osMailCreate(osMailQ(pwmIn_mailbox), NULL);
  analys_mailbox = osMailCreate(osMailQ(analys_mailbox), NULL);
  
  /* Start scheduler */
  osKernelStart(NULL, NULL);
  
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1)
  {
    /* NO CODE IN HERE! USE THE DEFAULT TASK OR CUSTOM MADE TASKS */
  }
} 
/* end of main */

#ifdef USE_FULL_ASSERT
/*
* Name:  assert_failed
* Brief: write a function brief here!
* Args:  write about function arguments here!
* Rets:  write about return values here!
*/ 
void assert_failed(uint8_t* file, uint32_t line)
{
  //TODO ?
}
#endif

/**********END OF FILE****/
