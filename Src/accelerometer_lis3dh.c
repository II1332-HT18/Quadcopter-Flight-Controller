/** ****************************************************************************
 * @file 
 *
 * @author Jacob Kimblad & Max Kufa
 * @version V1.1
 * @date 2015-12-10
 * @author Jonathan Lindberg
 * @version V2.0
 * @date 2017-11-24
 * 
 * @brief Contains accelerometer-related functionality
 *
 * @detail Contains functions to read data from the LIS3DH accelerometer.
 *
 * @see http://www.st.com/en/mems-and-sensors/lis3dh.html
 ******************************************************************************/

/* INCLUDE ********************************************************************/
#include "accelerometer_lis3dh.h"
#include "stm32f3xx_hal.h"
#include <math.h>

/* EXTERNAL DEFINITIONS *******************************************************/
extern SPI_HandleTypeDef hspi2;

/* PRIVATE DEFINES ************************************************************/
#define M_PI 3.14159265358979323846
#define cs_acc_Pin GPIO_PIN_0
#define cs_acc_GPIO_Port GPIOB


/* Struct declaration */
static ACC_TypeDef acc_values;

/* PRIVATE FUNCTIONS **********************************************************/
int16_t ACC_read_x(void);
int16_t ACC_read_y(void);
int16_t ACC_read_z(void);

/* FUNCTION DEFINITIONS *******************************************************/

/** ****************************************************************************
 * @brief Reads and updates accelerometer data
 *
 * @detail Will update the x-,y-,z- values in struct acc_values of type 
 * ACC_TypeDef from the newly read values.
 ******************************************************************************/
void ACC_update_xyz(){
  acc_values.x_raw = ACC_read_x();
  acc_values.y_raw = ACC_read_y();
  acc_values.z_raw = ACC_read_z();
}

/** ****************************************************************************
 * @brief Get address of struct
 *
 * @param ACC_TypeDef** acc_values_ptr The pointer whose address should be
 * updated.
 *
 * @detail Gets address of struct and sets the received pointer to point at said
 * struct.
 ******************************************************************************/
void ACC_get_address(ACC_TypeDef** acc_values_ptr) {
  *acc_values_ptr = &acc_values;
}

/** ****************************************************************************
 * @brief Get copy of accelerometers struct
 *
 * @param ACC_TypeDef* pointer. Pointer to which the values within the struct
 * should be copied
 *
 * @detail Gets raw values stored in the struct acc_values and store them in 
 * the struct the pointer points to.
 ******************************************************************************/
void ACC_get_struct(ACC_TypeDef* pointer) {
  pointer->x_raw = acc_values.x_raw;
  pointer->z_raw = acc_values.z_raw;
  pointer->y_raw = acc_values.y_raw;
}

/** ****************************************************************************
 * @brief Get pitch angle
 *
 * @param ACC_TypeDef* values Pointer to struct from which the pitch angle
 * should be calculated and then stored within
 *
 * @detail Calculates the pitch angle from raw accelerometer data values within 
 * a struct and then stores the calculated pitch angle in the same struct.
 *
 *@todo Currently unused and could be removed without loss of functionality.
 ******************************************************************************/
void ACC_get_pitch_angle(ACC_TypeDef* values)
{
  values->pitch = atan(values->x_raw / sqrt(pow(values->y_raw, 2) + pow(values->z_raw, 2))) *(180.0 / M_PI);
}

/** ****************************************************************************
 * @brief Get roll angle
 *
 * @param ACC_TypeDef* values Pointer to struct from which the roll angle
 * should be calculated and then stored within
 *
 * @detail Calculates the roll angle from raw accelerometer data values within 
 * a struct and then stores the calculated pitch angle in the same struct.
 *
 *@todo Currently unused and could be removed without loss of functionality.
 ******************************************************************************/
void ACC_get_roll_angle(ACC_TypeDef* values)
{
  values->roll = atan(values->y_raw / sqrt(pow(values->x_raw, 2) + pow(values->z_raw, 2)))*(180.0 / M_PI);
}

/** ****************************************************************************
 * @brief Read X from the accelerometer.
 *
 * @return int16_t The accelerometers x-axis value.
 *
 * @detail Reads the value of the x-axis from the accelerometer with uart 
 * serial communication.
 ******************************************************************************/
int16_t ACC_read_x(void)
{
  int16_t x_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //GET X_l
  address = 0xA8;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //Start End transmission
  x_Value = received[1];
  x_Value = x_Value<<8;
  
  //GET X_h
  address = 0xA9;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  return (x_Value | received[1]);
}

/** ****************************************************************************
 * @brief Read Y from the accelerometer.
 *
 * @return int16_t The accelerometers y-axis value.
 *
 * @detail Reads the value of the y-axis from the accelerometer with uart 
 * serial communication.
 ******************************************************************************/
int16_t ACC_read_y(void)
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
  y_Value = y_Value<<8;
  
  //GET y_h
  address = 0xAB;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  return (y_Value | received[1]);
}

/** ****************************************************************************
 * @brief Read Z from the accelerometer.
 *
 * @return int16_t The accelerometers z-axis value.
 *
 * @detail Reads the value of the z-axis from the accelerometer with uart 
 * serial communication.
 ******************************************************************************/
int16_t ACC_read_z(void)
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
  z_Value = z_Value<<8;
  
  //GET z_h
  address = 0xAD;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  return (z_Value | received[1]);
}