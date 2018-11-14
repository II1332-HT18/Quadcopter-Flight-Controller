/** ****************************************************************************
* @file    filter.c
* @author  Henrik Bjorklund, Therese Kennerberg, Jonathan Lindberg, Simon Strom
* @version v1.0
* @date   2017-12-08
* @brief   Functions to filter raw input data
*******************************************************************************/

/* INCLUDE ********************************************************************/
#include "main.h"
#include "sensor.h"
#include "accelerometer_lis3dh.h"
#include "gyroscope_l3gd20h.h"

/* PRIVATE VARIABLES **********************************************************/
extern UART_HandleTypeDef huart3;

/* PRIVATE FUNTIONS ***********************************************************/
void filter_lowpass(FILTER_lowpass_struct *val, float input);
void filter_complement(FILTER_complement_struct* complement_data);

/* FUNCTION DEFINITIONS *******************************************************/
/** ****************************************************************************
 * @brief Filtering task
 * 
 * @detail Thread that reads sensor data from the gyroscope and accelerometer,
 * applies a lowpass filter to the gyroscope data and then combines the
 * gyroscope and accelerometer data through a complement filter.
 ******************************************************************************/
void StartsensorFilterTask(void const * arguments)
{
  // Trace message
  //printf("StartsensorFilterTask has been created");
  HAL_GPIO_WritePin(GPIOC, 1, GPIO_PIN_SET); // Turn on LED1
  
  // Declare local variables for the task
  extern const portTickType MAIN_FREQUENCY;  
  portTickType  last_task_start = xTaskGetTickCount();
  
  // Initializing ACC raw value handle
  ACC_TypeDef acc_raw;
  
  // Initializing GYR raw value handle
  GYR_TypeDef gyr_raw;
  
  // Initializing lowpass filter data handle.
  FILTER_lowpass_struct lowpass_data_acc_x;
  FILTER_lowpass_struct lowpass_data_acc_y;
  FILTER_lowpass_struct lowpass_data_acc_z;
  
  // Initializing complement filter data handle
  FILTER_complement_struct complement_data;
  
  // Setting all lowpass filter parameters to zero.
  lowpass_data_acc_x.val1[0] = 0;
  lowpass_data_acc_x.val1[1] = 0;
  lowpass_data_acc_x.val1[2] = 0;
  lowpass_data_acc_x.val2[0] = 0;
  lowpass_data_acc_x.val2[1] = 0;
  lowpass_data_acc_x.val2[2] = 0;
  
  lowpass_data_acc_y.val1[0] = 0;
  lowpass_data_acc_y.val1[1] = 0;
  lowpass_data_acc_y.val1[2] = 0;
  lowpass_data_acc_y.val2[0] = 0;
  lowpass_data_acc_y.val2[1] = 0;
  lowpass_data_acc_y.val2[2] = 0;
  
  lowpass_data_acc_z.val1[0] = 0;
  lowpass_data_acc_z.val1[1] = 0;
  lowpass_data_acc_z.val1[2] = 0;
  lowpass_data_acc_z.val2[0] = 0;
  lowpass_data_acc_z.val2[1] = 0;
  lowpass_data_acc_z.val2[2] = 0;
  
  complement_data.acc_x         = 0;
  complement_data.acc_y         = 0;
  complement_data.acc_z         = 0;
  complement_data.gyr_x         = 0;
  complement_data.gyr_y         = 0;
  complement_data.gyr_z         = 0;
  complement_data.filter_pitch  = 0;
  complement_data.filter_roll   = 0;
  complement_data.filter_yaw    = 0;
  complement_data.acc_pitch = 0;
  complement_data.acc_roll  = 0;
  
  while(1)
  {
    // Preparing accelerometer data.
    ACC_update_xyz();           // Reading sensor data and storing to driver struct.
    ACC_get_struct(&acc_raw);   // Copying driver struct data to local data struct.
    
    // Preparing gyroscope data.
    GYR_update_xyz();
    GYR_get_struct(&gyr_raw);
    
    // Lowpass filter on ACC raw data
    filter_lowpass(&lowpass_data_acc_x, (float) acc_raw.x_raw);
    filter_lowpass(&lowpass_data_acc_y, (float) acc_raw.y_raw);
    filter_lowpass(&lowpass_data_acc_z, (float) acc_raw.z_raw);
    
    // Prepare complement_data struct.
    complement_data.acc_x = lowpass_data_acc_x.output;
    complement_data.acc_y = lowpass_data_acc_y.output;
    complement_data.acc_z = lowpass_data_acc_z.output;
    complement_data.gyr_x = gyr_raw.x_raw;
    complement_data.gyr_y = gyr_raw.y_raw;
    complement_data.gyr_z = gyr_raw.z_raw;
    
    // Run complement filter
    filter_complement(&complement_data);
    
    // Send data with UART.
    float val[11];
    uint8_t ctr = 0;
    val[ctr++] = xTaskGetTickCount();
    val[ctr++] = lowpass_data_acc_x.output;
    val[ctr++] = lowpass_data_acc_y.output;
    val[ctr++] = lowpass_data_acc_z.output; 
    val[ctr++] = acc_raw.x_raw;
    val[ctr++] = acc_raw.z_raw;
    val[ctr++] = acc_raw.y_raw;
    val[ctr++] = complement_data.filter_pitch;
    val[ctr++] = complement_data.filter_roll;
    val[ctr++] = complement_data.acc_pitch;
    val[ctr++] = complement_data.acc_roll;
      
    // Transmit signals to UART peripheral
    HAL_UART_Transmit(&huart3,(uint8_t*)&val, sizeof(float)*ctr, 4);
    huart3.State = HAL_UART_STATE_READY;

    // Send mail
    //  osMailPut(analys_mailbox, lowpass_data_acc_x);
    osMailPut(sensorFilter_mailbox, &complement_data);
    vTaskDelayUntil(&last_task_start,MAIN_FREQUENCY); 
    
  }
}
/** ****************************************************************************
 * @brief Butterworth lowpass filter
 * 
 * @param FILTER_lowpass_struct *axel Pointer to a struct for saving immediate
 * values
 * @param float input Latest input value for recursive filter
 *
 * @detail Return a value after it have been processed by a 2-order Butterworth
 * filter with cutoff frequency 45Hz.
 * ******************************************************************************/
void filter_lowpass(FILTER_lowpass_struct *IO_data, float input){
  // Initializing filter values.
  float x[3];
  float y[3];
  x[0] = input;
  x[1] = IO_data->val1[0];
  x[2] = IO_data->val1[1];
  y[1] = IO_data->val2[0];
  y[2] = IO_data->val2[1];
  
  // Constants from Matlab for 2-degree Butterworth filter cutoff = 45Hz, Fsample = 250 Hz
  float A1 = -0.2212; 
  float A2 = 0.1802;
  float B0 = 0.2398;
  float B1 = 0.4975;
  float B2 = 0.2398;
  
  // Lowpass filter function.
  y[0] = A1*y[1]+A2*y[2]+B0*x[0]+B1*x[1]+B2*x[2];
  
  // Storing all filter values.
  IO_data->val1[0] = x[0];
  IO_data->val1[1] = x[1];
  IO_data->val1[2] = x[2];
  IO_data->val2[0] = y[0];
  IO_data->val2[1] = y[1];
  IO_data->val2[2] = y[2];
  
  // Storing filtered IO_data value.
  IO_data->output = y[0]; 
}

/** ****************************************************************************
 * @brief Complement filter combining gyroscope and accelerometer data
 *
 * @param FILTER_complement_struct* complement_data Pointer to the struct from
 * which filtered gyroscope and accelerometer data is going be read and then
 * stored in.
 * 
 * @detail Combines accelerometer and filtered gyroscope data in order to
 * correct the gyroscope's drift over time using the accelerometer data.
 ******************************************************************************/
void filter_complement(FILTER_complement_struct* complement_data){
  float c = 0.98;       // Accelerometer weight constant.
  float dt = 0.004;     // Our sampling rate 1/250 Hz
  
  complement_data->filter_pitch += (complement_data->gyr_x * (GYRO_SENSITIVITY/MILLIDPS_TO_DPS))*dt; // Calculating pitch angle
  complement_data->filter_roll  += (complement_data->gyr_y * (GYRO_SENSITIVITY/MILLIDPS_TO_DPS))*dt; // Calculating roll angle
  complement_data->filter_yaw   += (complement_data->gyr_z * (GYRO_SENSITIVITY/MILLIDPS_TO_DPS))*dt; // Calculating yaw angle
  
  // Calculating pitch using formula arctan(x/sqrt(y^2+z^2))
  complement_data->acc_pitch = atan2f(complement_data->acc_x,sqrt(pow(complement_data->acc_y,2) + pow(complement_data->acc_z,2))) * 180 / M_PI;
  
  // Calculating roll using formula arctan(y/sqrt(x^2+z^2))
  complement_data->acc_roll  = atan2f(complement_data->acc_y,sqrt(pow(complement_data->acc_x,2) + pow(complement_data->acc_z,2))) * 180 / M_PI;
  
  //complement_data->filter_yaw  = atan2f(complement_data->acc_z,sqrt(pow(complement_data->acc_x,2) + pow(complement_data->acc_z,2))) * 180 / M_PI;
  
  // Using the equation angle=0.98(angle+gyroData*dt)+0.02accData
  complement_data->filter_pitch = c * complement_data->filter_pitch + complement_data->acc_pitch * (1 - c);
  complement_data->filter_roll  = c * complement_data->filter_roll  + complement_data->acc_roll  * (1 - c);
  //complement_data->filter_yaw  = c * complement_data->filter_yaw  + complement_data->acc_yaw  * (1 - c);
  
  //yaw = 180 * atan (accelerationZ/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
  //roll = 180 * atan (accelerationY/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
}







