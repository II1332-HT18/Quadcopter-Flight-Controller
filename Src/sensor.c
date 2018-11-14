/**
 * @file 
 *
 * @author Jacob Kimblad & Max Kufa
 * @version V1.11
 * @date 2015-12-10
 * 
 * @brief Contains sensor-related functionality
 *
 * @detail Contains functions to read data from the L3GD20H gyroscope and the 
 * LIS3DH accelerometer.
 *
 * @see http://www.st.com/en/mems-and-sensors/l3gd20h.html
 * @see http://www.st.com/en/mems-and-sensors/lis3dh.html
 *
 * @todo Remove the sensor.c file - It has been deprecated.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*External definitions*/
extern SPI_HandleTypeDef hspi2;

/**
 * @brief Reads sensor data
 * 
 * @param main_struct* Pointer to struct containing global values
 *
 * @detail
 *
 * @todo Rewrite to use new structs instead of main_struct.
 */
void readSensors(main_struct* all_values){
  
  if(all_values != NULL){
    all_values->x_acc = lis3dh_Read_X();
    all_values->y_acc = lis3dh_Read_Y();
    all_values->z_acc = lis3dh_Read_Z();
    
    //all_values->accel_pitch_angle.f = accelerometer_Get_Pitch(lis3dh_Read_X(), lis3dh_Read_Y(), lis3dh_Read_Z());
    all_values->accel_pitch_angle.f = accelerometer_Get_Pitch(all_values->x_acc, all_values->y_acc, all_values->z_acc);
    //all_values->accel_roll_angle.f = accelerometer_Get_Roll(lis3dh_Read_X(), lis3dh_Read_Y(), lis3dh_Read_Z());
    all_values->accel_roll_angle.f = accelerometer_Get_Roll(all_values->x_acc, all_values->y_acc, all_values->z_acc);
    
    all_values->gyro_pitch_rate.f = (float)l3gd20h_Read_X();
    all_values->gyro_roll_rate.f = (float)l3gd20h_Read_Y();
    all_values->previous_gyro_yaw_rate.f = all_values->gyro_yaw_rate.f; //Store previous gyro rate
    all_values->gyro_yaw_rate.f = (float)l3gd20h_Read_Z();
  }
  return;
}

/**
 * @brief Read X from gyroscope
 * 
 * @return int16_t The rotation speed
 *
 * @detail Reads the data from the l3gd20h gyroscope sensor and returns the
 * current rotation speed on the X axis.
 * 
 * @todo 1. Find where addresses are found.
 * @todo 2. Find where the magic scaling numbers 70/1000 is derived from.
 */
float l3gd20h_Read_X(void)
{
  //Function variables
  int16_t x_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //Read from x_low register
  address = 0xA8; //Todo 1
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  x_Value = received[1];
  x_Value = x_Value << 8; //recieved bit are not correctly aligned. Shift them 8 bits.
  
  //Read from x_high register
  address = 0xA9; //Todo 1
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);//Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);//End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (float)(x_Value | received[1])*70/1000; //Todo 2
}

/**
 * @brief Read Y from gyroscope
 * 
 * @return int16_t The rotation speed
 *
 * @detail Reads the data from the l3gd20h gyroscope sensor and returns the
 * current rotation speed on the Y axis.
 * 
 * @todo 1. Find where addresses are found.
 * @todo 2. Find where the magic scaling numbers 70/1000 is derived from.
 */
float l3gd20h_Read_Y(void)
{
  //Function variables
  int16_t y_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //Read from y_l register
  address = 0xAA;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  y_Value = received[1];
  y_Value = y_Value << 8; //Shift 8 bits
  
  //Read from y_h register
  address = 0xAB;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (float)(y_Value | received[1])*70/1000;
}


/**
 * @brief Read Z from gyroscope
 * 
 * @return int16_t The rotation speed
 *
 * @detail Reads the data from the l3gd20h gyroscope sensor and returns the
 * current rotation speed on the Z axis.
 * 
 * @todo 1. Find where addresses are found.
 * @todo 2. Find where the magic scaling numbers 70/1000 is derived from.
 */
float l3gd20h_Read_Z(void)
{
  //Function variables
  int16_t z_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //Read from z_l register
  address = 0xAC;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  z_Value = received[1];
  z_Value = z_Value << 8; //Shift 8 bits
  
  //Read from z_h register
  address = 0xAD;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (float)(z_Value | received[1])*70/1000;
}

/**
 * @brief Read Force X from accelerometer
 * 
 * @return int16_t Force in the X direction
 *
 * @detail Reads the data from the lis3dh accelerometer sensor and returns the
 * current force in the X direction.
 * 
 * @todo 1. Find where addresses are found.
 * @todo 2. Find where the magic scaling numbers 1/16.384 is derived from.
 */
float lis3dh_Read_X(void)
{
  int16_t x_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //Read from x_l register
  address = 0xA8;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  x_Value = received[1];
  x_Value = x_Value*256;         //Shift left 8 bits into 16bit int 
  
  //Read from x_h register
  address = 0xA9;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (x_Value | received[1])/16.384;       
}

/**
 * @brief Read Force Y from accelerometer
 * 
 * @return int16_t Force in the Y direction
 *
 * @detail Reads the data from the lis3dh accelerometer sensor and returns the
 * current force in the Y direction.
 * 
 * @todo 1. Find where addresses are found.
 * @todo 2. Find where the magic scaling numbers 1/16.384 is derived from.
 */
float lis3dh_Read_Y(void)
{
  int16_t y_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //GET y_l
  address = 0xAA;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //Start End transmission
  y_Value = received[1];
  y_Value = y_Value*256;         //Shift 8 bits
  
  //GET y_h
  address = 0xAB;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (y_Value | received[1])/16.384;
}

/**
 * @brief Read Force Z from accelerometer
 * 
 * @return int16_t Force in the Z direction
 *
 * @detail Reads the data from the lis3dh accelerometer sensor and returns the
 * current force in the Z direction.
 * 
 * @todo 1. Find where addresses are found.
 * @todo 2. Find where the magic scaling numbers 1/16.384 is derived from.
 */
float lis3dh_Read_Z(void)
{
  int16_t z_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //GET z_l
  address = 0xAC;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  z_Value = received[1];
  z_Value = z_Value*256;         //Shift 8 bits
  
  //GET z_h
  address = 0xAD;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (z_Value | received[1])/16.384;
}

/**
 * @brief Get pitch value
 * 
 * @param float x_Acceleration X Acceleration
 * @param float y_Acceleration Y Acceleration
 * @param float z_Acceleration Z Acceleration
 *
 * @return float Pitch
 * 
 * @detail Transform the forces in all the axes to degrees in pitch
 *
 * @todo Figure out if we can switch to atan2 from atan or not.
 * @todo Rename M_PI to something more understandable.
 */
float accelerometer_Get_Pitch(float x_Acceleration, float y_Acceleration, float z_Acceleration)
{
  //Function variables
  float pitch;
  //Mathematical calculation for PITCH  
  pitch = atan(x_Acceleration/sqrt(pow(y_Acceleration,2) + pow(z_Acceleration,2)));
  
  //Turn into degrees
  return pitch*(180.0/M_PI);
}

/**
 * @brief Get roll value
 * 
 * @param float x_Acceleration X Acceleration
 * @param float y_Acceleration Y Acceleration
 * @param float z_Acceleration Z Acceleration
 *
 * @return float Roll
 * 
 * @detail Transform the forces in all the axes to degrees in roll
 *
 * @todo Figure out if we can switch to atan2 from atan or not.
 * @todo Rename M_PI to something more understandable.
 */
float accelerometer_Get_Roll(float x_Acceleration, float y_Acceleration, float z_Acceleration)
{
  //Function variables
  float roll;
  //Mathematical calculation for ROLL
  roll = atan(y_Acceleration/sqrt(pow(x_Acceleration,2) + pow(z_Acceleration,2)));
  
  //Turn into degrees
  return roll*(180.0/M_PI);
}