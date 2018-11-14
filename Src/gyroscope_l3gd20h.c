/** ****************************************************************************
 * @file 
 *
 * @author Jacob Kimblad & Max Kufa
 * @version V1.1
 * @date 2015-12-10
 * @author Therese Kennerberg
 * @version V2.0
 * @date 2017-11-24
 * 
 * @brief Contains gyroscope-related functionality
 *
 * @detail Contains functions to read data from the L3GD20H gyroscope.
 *
 * @see http://www.st.com/en/mems-and-sensors/l3gd20h.html
 ******************************************************************************/
   
#include "gyroscope_l3gd20h.h"
#include "stm32f3xx_hal.h"

/* Struct declaration */
static GYR_TypeDef gyr_values;

/* Function prototype*/
int16_t GYR_read_x(void);
int16_t GYR_read_y(void);
int16_t GYR_read_z(void);

/*External definitions*/
extern SPI_HandleTypeDef hspi2;

/** ****************************************************************************
 * @brief Reads and updates gyroscope sensor data
 *
 * @detail Read the signal from the gyroscope and and store them in the gyro
 * data struct.
 ******************************************************************************/
void GYR_update_xyz(void){
  gyr_values.x_raw = GYR_read_x();
  gyr_values.y_raw = GYR_read_y();
  gyr_values.z_raw = GYR_read_z();
}

/** ****************************************************************************
 * @brief Get copy of struct
 * 
 * @param GYR_TypeDef* pointer Pointer to which the values within the struct
 * should be copied
 *
 * @detail Takes a pointer to a GYR_TypeDef struct after which said struct is
 * filled with current gyroscope sensor data.
 ******************************************************************************/
void GYR_get_struct(GYR_TypeDef* pointer) {
  pointer->x_raw = gyr_values.x_raw;
  pointer->z_raw = gyr_values.z_raw;
  pointer->y_raw = gyr_values.y_raw;
}

/** ****************************************************************************
 * @brief Read X from gyroscope
 * 
 * @return int16_t The rotation speed (digits)
 *
 * @detail Reads the data from the l3gd20h gyroscope sensor and returns the
 * current rotation speed in the "digit" unit on the X axis. (1 digit = 
 * millidps*70 with the currently selected scale and sensitivity)
 ******************************************************************************/
int16_t GYR_read_x(void)
{
  // Function variables
  int16_t x_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  // Read from x_low register
  address = 0xA8; 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); // Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); // End SPI transmission
  x_Value = received[1];
  x_Value = x_Value << 8; // Recieved bit are not correctly aligned. Shift them 8 bits.
  
  // Read from x_high register
  address = 0xA9; 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);// Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);// End SPI transmission
  x_Value |= received[1];
  
  return x_Value;
}

/** ****************************************************************************
 * @brief Read Y from gyroscope
 * 
 * @return int16_t The rotation speed (digits)
 *
 * @detail Reads the data from the l3gd20h gyroscope sensor and returns the
 * current rotation speed in the "digit" unit on the Y axis. (1 digit = 
 * millidps*70 with the currently selected scale and sensitivity)
 ******************************************************************************/
int16_t GYR_read_y(void)
{
  // Function variables
  int16_t y_Value = 0;
  uint8_t address;
  uint8_t received[2];

  // Read from y_l register
  address = 0xAA;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); // Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); // End SPI transmission
  y_Value = received[1];
  y_Value = y_Value << 8; // Shift 8 bits
  
  // Read from y_h register
  address = 0xAB;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); // Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); // End SPI transmission
  // Add remaining 8 bits into the 16 bit int and return
  y_Value |= received[1];

  return y_Value;
}


/** ****************************************************************************
 * @brief Read Z from gyroscope
 * 
 * @return int16_t The rotation speed (digits)
 *
 * @detail Reads the data from the l3gd20h gyroscope sensor and returns the
 * current rotation speed in the "digit" unit on the Z axis. (1 digit = 
 * millidps*70 with the currently selected scale and sensitivity)
 ******************************************************************************/
int16_t GYR_read_z(void)
{
  // Function variables
  int16_t z_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  // Read from z_l register
  address = 0xAC;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); // Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); // End SPI transmission
  z_Value = received[1];
  z_Value = z_Value << 8; //Shift 8 bits
  
  // Read from z_h register
  address = 0xAD;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); // Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); // End SPI transmission
  // Add remaining 8 bits into the 16 bit int and return
  z_Value |= received[1];
  
  return z_Value;
}
