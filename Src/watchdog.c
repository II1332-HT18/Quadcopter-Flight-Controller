/**
******************************************************************************
* @file    watchdog.c
* @author  Ramón Nordin Rodriguez
* @version version number here
* @date    -
* @brief   The WD is implemented in hardware, the loadhandler task checks if any 
* thread is currently unresponsive. If so, it attempts a reset of the thread, 
* before kicking the WD. If a kick is sent in time, then nemas problemas hombre, 
* else we get a system reset.
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "watchdog.h"
#include <string.h>

/* Stuff that should get sorted ----------------------------------------------*/
//WD handle
IWDG_HandleTypeDef hiwdg;

/*helper functions for the WD */
//macro that sends a kick to the WD
#define KICK_WATCHDOG hiwdg.Instance->KR=0xAAAA

/* Function definitions ------------------------------------------------------*/
/**
 * @brief Fast strcpy function, caller is responsible for memory management
 *  
 * @param char pointer destination
 * @param char pointer source
 *
 */
void string_copy(char* destination,char* source){
  while(*source!='\0'){
    *destination++=*source++;
  }*destination='\0';
}

/**
 * @brief Copys all fields from const struct os_thread_def into a user defined struct
 *        that isnt const. Simplyfies implementation of loadhandler. 
 *        This function is called from main following the creation of each thread.
 *
 * @param threadDef pointer destination
 * @param osThreadDef_t pointer source
 *  
 */
void* copy_osThreadDef_UD_t(threadDef* destination, osThreadDef_t* source){

 //the maxlen of task name is set to 25, default in FreeRTOS is 16
 //this could be optimized later
 if((destination->name = (char*) pvPortMalloc(25*sizeof(char)))==NULL)
   while(1);    //TODO - fix error handler
 
 string_copy((destination->name),(source->name));
 destination->pthread   = source->pthread;
 destination->tpriority = source->tpriority;
 destination->instances = source->instances;
 destination->stacksize = source->stacksize;
 
 return destination;
}

/**
 * @brief Setup for 200 ms reset
 * 
 */
void MX_IWDG_Init(void) //setup for 200ms resetv
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 250;
  hiwdg.Init.Reload = 250;
  HAL_IWDG_Init(&hiwdg);
}

//ugly af but a lack of time dictates desperate measures
//I am ofc refering to the use of globals
extern uint32_t gMainloopWDCheckback;
extern uint32_t gAnalysWDCheckback;
extern uint32_t gPwmInWDCheckback;

/**
 * @brief Loadhandler thread and monitor task
 * 
 * @param void const* arguments
 */
void loadHandleThread(void const* arguments){   //loadhandler thread, monitors task 
  threadInfo* tInfo = (threadInfo*)arguments;
  size_t size = tInfo->size;
  
  
  bool mainClear=false;  
  bool pwmClear=false;
  bool anaClear=false;
  osDelay(5500);
  MX_IWDG_Init();
  
 while(1){
    mainClear = (gMainloopWDCheckback>9);
    pwmClear  = (gPwmInWDCheckback>5);
    anaClear  = (gAnalysWDCheckback>3); 
    
    //all tasks have responded
  if(mainClear && pwmClear && anaClear){
    gMainloopWDCheckback=0;
    gPwmInWDCheckback=0;
    gAnalysWDCheckback=0;
    KICK_WATCHDOG;
  }
  //shit probally hit the fan
  else{ 
    if(!mainClear){vTaskDelete(tInfo->ID[0]);osThreadCreate((osThreadDef_t*)&(tInfo->tDef[0]),(void*)true);}
    if(!pwmClear) {vTaskDelete(tInfo->ID[1]);osThreadCreate((osThreadDef_t*)&(tInfo->tDef[1]),*(tInfo->arg));}
    if(!anaClear) {vTaskDelete(tInfo->ID[2]);osThreadCreate((osThreadDef_t*)&(tInfo->tDef[2]),*(tInfo->arg));}    
    }    
  osDelay(40);
  }
}
