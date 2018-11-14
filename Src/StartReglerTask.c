
#include "StartReglerTask.h"

void StartReglerTask(void const * arguments)
{

  /*Declare local variables for the task*/
   const portTickType FILTER_FREQUENCY = 10;  
  
  portTickType  last_task_start = xTaskGetTickCount();
  

  
  float gyro_pitch=0;
  float acc_pitch=0;
  float gyro_roll=0;
  float acc_roll=0; 
  
  /* Create an event to check mailboxes */
   osEvent check_mailbox_event;
  
  /* Main loop */
  while(1)
  {
    /* Allocate memory for mailbox */
    //sensor_values_struct *sensor_values =(sensor_values_struct*)osMailAlloc(sensor_mailbox,osWaitForever);
    
    
    /*
    
    Read from the sensor_mailbox 
    
    */
    
      /* Create an event to check the filter_mailbox */
    check_mailbox_event= osMailGet(filter_mailbox, osWaitForever);
     
      /* Check if there is anything in the mailbox */
    if(check_mailbox_event.status == osEventMail)
    {
      /* create a local pointer to the struct in the mailbox */
      filter_values_struct *filter_values =(filter_values_struct*)check_mailbox_event.value.p;
    
     
      /*Code*/
      /* Save data from struct to local variables */
      
      gyro_pitch = filter_values->filtered_pitch_angle;
      
      /*
      acc_pitch = sensor_values->accel_pitch_angle;
      gyro_roll = sensor_values->gyro_roll_rate;
      acc_roll = sensor_values->accel_roll_angle;
      */
      HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
   //   osDelay(500);
      HAL_Delay(300);     // Delay without "yield"
      
      
      /* Free allocated memory when no more readings will be done */
     osMailFree(filter_mailbox, filter_values);
    /* Delay the task to a fixed time to ensure constant execution frequncy */
    //vTaskDelayUntil(&last_task_start,FILTER_FREQUENCY);
    
      
    
    /*We are done in this thread so we yield*/
    osThreadYield();
     //osDelay(1);
     
    }
    
    
    
     /************************************************************************ 
      
      Read from the pwmIn_mailbox
      
      */
      
      /* Create an event to check the pwmIn_mailbox */
    check_mailbox_event = osMailGet(pwmIn_mailbox, osWaitForever);
     
      /* Check if there is anything in the mailbox */
      if(check_mailbox_event.status == osEventMail)
      {
      /* create a local pointer to the struct in the mailbox */
     pwmIn_struct *pwmIn_values =(pwmIn_struct*)check_mailbox_event.value.p;
     
      
      
      /*Code*/
      
     HAL_GPIO_TogglePin(GPIOC, LED2_Pin);
  // osDelay(500);
     // HAL_Delay(200);     // Delay without "yield"s
      /* Save data from struct to local variables */
      
      /* Free allocated memory when no more readings will be done */
     osMailFree(pwmIn_mailbox, pwmIn_values);
     
     }
      
      
      
    /* Delay the task to a fixed time to ensure constant execution frequncy */
  //  vTaskDelayUntil(&last_task_start,FILTER_FREQUENCY);
    
      
    
    /*We are done in this thread so we yield*/
    osThreadYield(); 
    //osDelay(1);
    
      }
  }
