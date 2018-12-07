/** ****************************************************************************
* @file    filter.c
*
* @author  Alpha Fofana, Carl Mossberg, Simon Lagerqvist, Tommie Höglund Gran
* @version 1.1
* @date    2018-11-22
*
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
extern const portTickType HYPERPERIOD;

/* PRIVATE FUNTIONS ***********************************************************/
void filter_lowpass(FILTER_lowpass_struct *val, float input);
void filter_complement(FILTER_complement_struct* complement_data);
float angle_speed_calculator(FILTER_angle_speed_struct *history,float current_value);

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
  
  // Initializing angle speed, the derivative, data handle
  FILTER_angle_speed_struct angle_speed_x;
  FILTER_angle_speed_struct angle_speed_y;
  FILTER_angle_speed_struct angle_speed_z;
  
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
  
  //Setting main struct values to zero
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
  //Added 2018-12-06 by CM
  complement_data.raw_acc_x = 0;
  complement_data.raw_acc_y = 0;
  complement_data.raw_acc_z = 0;
  complement_data.pitch_angle_speed = 0;
  complement_data.roll_angle_speed = 0;  
  complement_data.yaw_angle_speed = 0;
  
  //Setting angle speed struct values to zero
//  angle_speed_x.current = 0; //I dont need theses cuurent values, do I?
  angle_speed_x.old = 0;
//  angle_speed_y.current = 0; //I dont need theses cuurent values, do I?
  angle_speed_y.old = 0;
//  angle_speed_z.current = 0; //I dont need theses cuurent values, do I?
  angle_speed_z.old = 0;

  
  
  while(1)
  {
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

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
    complement_data.gyr_x = -gyr_raw.x_raw; //Correcting the angle
    complement_data.gyr_y = -gyr_raw.y_raw; //Correcting the angle
    complement_data.gyr_z = -gyr_raw.z_raw; //Correcting the angle
    
    //Below created to access the RAW acc data to be able ro analyze
    complement_data.raw_acc_x = acc_raw.x_raw;  //the actual RAW acc_x
    complement_data.raw_acc_y = acc_raw.y_raw;  //the actual RAW acc_y
    complement_data.raw_acc_z = acc_raw.z_raw;  //the actual RAW acc_z
    
    // Run complement filter
    filter_complement(&complement_data);
    
    //Create angle speed values and store in main struct
    complement_data.pitch_angle_speed = angle_speed_calculator(&angle_speed_x, complement_data.filter_pitch);
    complement_data.roll_angle_speed = angle_speed_calculator(&angle_speed_y, complement_data.filter_roll);
    complement_data.yaw_angle_speed = angle_speed_calculator(&angle_speed_z, complement_data.filter_yaw);
   

    // Send mail
    //  osMailPut(analys_mailbox, lowpass_data_acc_x); //removed by ??
    osMailPut(sensorFilter_mailbox, &complement_data);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    vTaskDelayUntil(&last_task_start,HYPERPERIOD/5); 
    
  }
}

/** ****************************************************************************
 * @brief Simple function to calculate difference a.k.a. change speed of input
 * 
 * @param FILTER_angle_speed_struct history Pointer to a struct for saving old
 * values
 * @param float current_value The current value
 *
 * @return Float The difference between the old value and the new.
 * @detail Function used to calculate the difference between the current value of (x,y,z)
 * compared to the old value (x,y,z) stored in a struct. This in order to provide control
 * crew with good angle-speed to use in PID controller
 * ******************************************************************************/
float angle_speed_calculator(FILTER_angle_speed_struct *history,float current_value){
  float calculated_angle_speed;
  float dt = 0.004;     // Our sampling rate 1/250 Hz
  
  calculated_angle_speed = (current_value - history->old)/dt; //Calculate the difference
  history->old = current_value; //Set the current value to the new old
  return calculated_angle_speed;
}


/** ****************************************************************************
 * @brief Butterworth lowpass filter
 * 
 * @param FILTER_lowpass_struct *IO_data Pointer to a struct for saving immediate
 * values
 * @param float input Latest input value for recursive filter
 *
 * @detail Return a value after it have been processed by a 2-order Butterworth
 * filter with cutoff frequency XX Hz.
 * ******************************************************************************/
void filter_lowpass(FILTER_lowpass_struct *IO_data, float input){
  // Initializing filter values.
  // Added more "degrees" for the Butterworth filter
  // Maybe, we won't need them, remove them in that case!
  float x[5]; // Added 2018-12-06, 3 -> 5
  float y[5]; // Added 2018-12-06, 3 -> 5
  x[0] = input;
  x[1] = IO_data->val1[0];
  x[2] = IO_data->val1[1];
  x[3] = IO_data->val1[2]; // Added 2018-12-06
  x[4] = IO_data->val1[3]; // Added 2018-12-06
  y[1] = IO_data->val2[0];
  y[2] = IO_data->val2[1];
  y[3] = IO_data->val2[2]; // Added 2018-12-06
  y[4] = IO_data->val2[3]; // Added 2018-12-06
  
  // Constants from Matlab for 2-degree Butterworth filter cutoff = 55Hz, Fsample = 250 Hz
//  float A1 = -0.2212; 
//  float A2 = 0.1802;
//  float B0 = 0.2398;
//  float B1 = 0.4975;
//  float B2 = 0.2398;
  
//45Hz WORKS WOHO
//  float A1 = -0.5193; 
//  float A2 = 0.2197;
//  float B0 = 0.1751;
//  float B1 = 0.3502;
//  float B2 = 0.1751; 
  
  //40Hz WORKS WOHO
//  float A1 = -0.6710; 
//  float A2 = 0.2523;
//  float B0 = 0.1453;
//  float B1 = 0.2906;
//  float B2 = 0.1453;  
  
    //35Hz FUNKAR  2018-12-06 working since we changed the filter (the As should be negative)
//  float A1 = -0.8252; 
//  float A2 = 0.2946;
//  float B0 = 0.1174;
//  float B1 = 0.2347;
//  float B2 = 0.1174;
  
   //15Hz, added after we realised that the filtercoeff A should be negative
 float A1 = -1.4755;
 float A2 = 0.5869;
 float B0 = 0.0279;
 float B1 = 0.0557;
 float B2 = 0.0279;
  
  
  // Lowpass filter function.
   y[0] = -A1*y[1]-A2*y[2]+B0*x[0]+B1*x[1]+B2*x[2]; //The old one 2nd degree
  // y[0] = A1*y[1]+A2*y[2]+A3*y[3]+A4*y[4]+B0*x[0]+B1*x[1]+B2*x[2]+B3*x[3]+B4*x[4]; //4th degree, added 2018-12-06
  
//  if  (isinf(y[0]))
//    printf("We are ****");  
  
  // Storing all filter values.
  IO_data->val1[0] = x[0];
  IO_data->val1[1] = x[1];
  IO_data->val1[2] = x[2];
  IO_data->val1[3] = x[3]; // Added 2018-12-06, if you want to use higher degree filter
  IO_data->val1[4] = x[4]; // Added 2018-12-06, if you want to use higher degree filter
  IO_data->val2[0] = y[0];
  IO_data->val2[1] = y[1];
  IO_data->val2[2] = y[2];
  IO_data->val2[3] = y[3]; // Added 2018-12-06, if you want to use higher degree filter
  IO_data->val2[4] = y[4]; // Added 2018-12-06, if you want to use higher degree filter
  
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







