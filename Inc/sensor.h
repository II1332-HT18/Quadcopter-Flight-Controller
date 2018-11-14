/*******************************sensor.h***************************************/

#ifndef __SENSOR_H
#define __SENSOR_H


/* Includes ------------------------------------------------------------------*/
#include "math.h"

/* Private defines -----------------------------------------------------------*/
#define M_PI 3.14159265358979323846
#define cs_acc_Pin GPIO_PIN_0
#define cs_acc_GPIO_Port GPIOB
#define cs_gyro_Pin GPIO_PIN_1
#define cs_gyro_GPIO_Port GPIOB
#define ENABLE_READ_SENSOR 0x80 //Enable read on gyroscope & accelerometer

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Private Variables----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void readSensors(main_struct*);

//Gyroscope
//static void l3gd20h_Read(void);
float l3gd20h_Read_X(void);
float l3gd20h_Read_Y(void);
float l3gd20h_Read_Z(void);
//Accelerometer
//static void lis3dh_Read(void);
float lis3dh_Read_X(void);
float lis3dh_Read_Y(void);
float lis3dh_Read_Z(void);
float accelerometer_Get_Pitch(float, float, float);
float accelerometer_Get_Roll(float, float, float);
#endif