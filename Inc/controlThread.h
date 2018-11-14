/********************controlThread.h******************************/

#ifndef __CONTROLTHREAD_H
#define __CONTROLTHREAD_H

#include "main.h"
/* Functions -----------------------------------*/
void getPWMinValues(main_struct* all_values);
void StartControlTask(void const * arguments);


/* Private defines -----------------------------------------------------------*/
/*
#define M_PI 3.14159265358979323846
#define cs_acc_Pin GPIO_PIN_0
#define cs_acc_GPIO_Port GPIOB
#define cs_gyro_Pin GPIO_PIN_1
#define cs_gyro_GPIO_Port GPIOB
#define BUFFERSIZE 8
*/

#endif /* __SENSORER_H */