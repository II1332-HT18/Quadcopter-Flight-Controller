/* gyroscope_l3gd20h.h */


#ifndef __GYROSCOPE_L3GD20H_H
#define __GYROSCOPE_L3GD20H_H

/* DEFINES ********************************************************************/
#define GYRO_SENSITIVITY        70.0
#define MILLIDPS_TO_DPS         1000.0

/* INCLUDES *******************************************************************/
#include "stm32f3xx_hal.h"

/*Struct to acces gyroscopes value*/
typedef struct{
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
} GYR_TypeDef;

/*Macros or private defines*/
#define cs_gyro_Pin GPIO_PIN_1
#define cs_gyro_GPIO_Port GPIOB
#define ENABLE_READ_SENSOR 0x80 //Enable read on gyroscope & accelerometer

void GYR_update_xyz();
void GYR_get_struct(GYR_TypeDef* pointer);

#endif