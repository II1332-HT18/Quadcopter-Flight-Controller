/**
******************************************************************************
* @file    automaticControl.c
* @author  Michael Fransson, Michael Henriksson, Ali Qhorbani, Daniel Hooshidar, 
*          Alexander Vassiliou, Markus Brislov, David Nordberg
* @date    11 December 2018
* @brief   This file contains the PID control system that stabilizes flight.
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Filter.h"

/* External variables --------------------------------------------------------*/
extern TIM_OC_InitTypeDef sConfigOC;
extern TIM_HandleTypeDef htim2;

/* Private variables ---------------------------------------------------------*/

/* Constants */
const float dt = 0.01;

const float WINDUP_ROLL_MAX = 10;
const float WINDUP_PITCH_MAX = 10;
const float WINDUP_YAW_MAX = 10;

const int GYRO_COMPUTE_BIAS_SAMPLE_SIZE = 1000;

/* PID variables ----------------*/
static float GYRO_BIAS_X = 0;
static float GYRO_BIAS_Y = 0;
static float GYRO_BIAS_Z = 0;

/* Roll */
static float roll_kp = 0.0858   + 0.5;
static float roll_ki = 0.112157 + 0.0;
static float roll_kd = 0.043758 - 0.01;
static float filtered_roll_angle;
static float desired_roll_angle = 0;
static float errorRoll = 0;
static float derivateRoll = 0;
static float integralRoll = 0;
static float PIDoutputRoll = 0;

/* Pitch */
static float pitch_kp = 0.282    + 0.1;
static float pitch_ki = 0.261111 + 0.0;
static float pitch_kd = 0.07614  + 0.0;
static float filtered_pitch_angle;
static float desired_pitch_angle = 0;
static float errorPitch = 0;
static float derivatePitch = 0;
static float integralPitch = 0;
static float PIDoutputPitch = 0;

/* Yaw */
static float yaw_kp = 0;
static float yaw_ki = 0;
static float yaw_kd = 0;
static float filtered_yaw_angle;
static float desired_yaw_angle = 0;
static float errorYaw = 0;
static float derivateYaw = 0;
static float integralYaw = 0;
static float PIDoutputYaw = 0;

/* Motors */
static int ThrustOnMotor = 0;
static int EmergencyValue = 0;
static int RFmotor = 0;
static int LFmotor = 0;
static int RBmotor = 0;
static int LBmotor = 0;

/******************************************************************************
* @brief Initialize the automatic PID control system.
* 
* @param void
* 
* @return void
******************************************************************************/
void automaticControl_init()
{
  FILTER_complement_struct *filterPointer;
  osEvent filterMail;
  float gyro_bias_x;
  float gyro_bias_y;
  float gyro_bias_z;
  
  gyro_bias_x = 0;
  gyro_bias_y = 0;
  gyro_bias_z = 0;
  
  for (int i = 0; i < GYRO_COMPUTE_BIAS_SAMPLE_SIZE; i++) {
    filterMail = osMailGet(sensorFilter_mailbox, osWaitForever);
    filterPointer = (FILTER_complement_struct*)filterMail.value.p;
    
    gyro_bias_x += filterPointer->gyr_x / GYRO_COMPUTE_BIAS_SAMPLE_SIZE;
    gyro_bias_y += filterPointer->gyr_y / GYRO_COMPUTE_BIAS_SAMPLE_SIZE;
    gyro_bias_z += filterPointer->gyr_z / GYRO_COMPUTE_BIAS_SAMPLE_SIZE;
    
    osMailFree(sensorFilter_mailbox, filterPointer);
  }
  
  GYRO_BIAS_X = gyro_bias_x;
  GYRO_BIAS_Y = gyro_bias_y;
  GYRO_BIAS_Z = gyro_bias_z;
}

/*******************************************************************************
* @brief Main function for the automatic PID control of the quadcopter's orientation.
* 
* @param all_values A pointer to the struct containing all global variables.
* 
* @return void
******************************************************************************/
void automaticControl(main_struct* all_values)
{
  /* Get mail */
  osEvent check_mail = osMailGet(pwmIn_mailbox, osWaitForever);
  pwmIn_struct *pwm_pointer = (pwmIn_struct*)check_mail.value.p;
  
  check_mail = osMailGet(sensorFilter_mailbox, osWaitForever);
  FILTER_complement_struct *filter_pointer = (FILTER_complement_struct*)check_mail.value.p;
  
    /* Actual value for Yaw, Pitch and Roll (angle rates from sensor) */
  filtered_yaw_angle = filter_pointer->filter_yaw;
  filtered_pitch_angle = filter_pointer->filter_pitch;
  filtered_roll_angle = filter_pointer->filter_roll;
  
  /* Setpoint for Yaw, Pitch and Roll */
  desired_yaw_angle = pwm_pointer->yaw;
  desired_pitch_angle = pwm_pointer->pitch;
  desired_roll_angle = pwm_pointer->roll;
  
  /* Remote control value limit control */
  remoteControlLimit();

  /*Run PID algoritm for Yaw, Pitch and Roll*/
  PID_Yaw(filter_pointer);
  PID_Pitch(filter_pointer);
  PID_Roll(filter_pointer);
  
  /* Motor control */
  ThrustOnMotor = pwm_pointer->thrust;
  EmergencyValue = pwm_pointer->emergency;
  motorControl();
  
  /* Emergency stop/No throttle and cap for velocity values that are out of bound */
  if((EmergencyValue != 0) || (ThrustOnMotor < 1100))
    emergencyStop(); // Emergency Stop & No throttle control
  
  /* Send motor values to motors. */
  changeVelocityOnMotorsWithPulseWidth(RFmotor, 1); // Right forward motor 
  changeVelocityOnMotorsWithPulseWidth(LFmotor, 3); // Left forward motor 
  changeVelocityOnMotorsWithPulseWidth(RBmotor, 2); // Right back motor 
  changeVelocityOnMotorsWithPulseWidth(LBmotor, 4); // Left back motor 
   
  //pull all in  all_values
  all_values->PIDoutputGyroYaw.f = PIDoutputYaw;      //Change name of errorgyroyaw
  all_values->PIDoutputPitch.f = PIDoutputPitch;
  all_values->PIDoutputRoll.f= PIDoutputRoll;
  
  all_values->RFmotor.f = RFmotor;
  all_values->LFmotor.f = LFmotor;
  all_values->RBmotor.f = RBmotor;
  all_values->LBmotor.f = LBmotor;
  
  all_values->yaw.f = desired_yaw_angle;
  all_values->pitch.f = desired_pitch_angle;
  all_values->roll.f = desired_roll_angle;
  all_values->thrust = ThrustOnMotor;
  all_values->emergency = EmergencyValue;
  
  osMailPut(analys_mailbox, all_values);
  
  /* Free memory occupied by mail */
  osMailFree(pwmIn_mailbox, pwm_pointer);
  osMailFree(sensorFilter_mailbox, filter_pointer);
}

/* Function definitions ------------------------------------------------------*/

/*******************************************************************************
* @brief Emergency stop to turn off motors.
*
* @param void
* 
* @return void
* 
* @detail If controller is in emergency stop mode, or thrust is at 0z then bypass PID.
*         Sets motor values to 1000 and disables PID calculations.
******************************************************************************/
void emergencyStop(void)
{
    RFmotor = 1000;
    LFmotor = 1000;
    RBmotor = 1000;
    LBmotor = 1000;
    
    // Yaw
    integralYaw = 0;
    derivateYaw = 0;
    
    //Pitch
    integralPitch = 0;
    derivatePitch = 0;
    
    //Roll
    integralRoll = 0;
    derivateRoll = 0;
}

/* Function definitions ------------------------------------------------------*/

/*******************************************************************************
* @brief  Remote control value limit control.
* 
* @param  void
* 
* @return void
******************************************************************************/
void remoteControlLimit()
{
  if (desired_roll_angle > 25)       desired_roll_angle = 25;
  else if (desired_roll_angle < -25) desired_roll_angle = -25;
  
  if (desired_pitch_angle > 25)       desired_pitch_angle = 25;
  else if (desired_pitch_angle < -25) desired_pitch_angle = -25;
  
  if (desired_yaw_angle > 25)       desired_yaw_angle = 25;
  else if (desired_yaw_angle < -25) desired_yaw_angle = -25;
}

/*******************************************************************************
* @brief PID control for Yaw.
* 
* @param filter_pointer A pointer to struct containing filter variables.
* 
* @return void
******************************************************************************/
void PID_Yaw(FILTER_complement_struct *filter_pointer)
{
    errorYaw    = desired_yaw_angle - filtered_yaw_angle;
    derivateYaw = 0 - (filter_pointer->gyr_z - GYRO_BIAS_Z);

    if (abs((int)integralPitch) <= WINDUP_YAW_MAX) integralYaw += errorYaw*dt;
    else if (integralYaw > 0)                      integralYaw = WINDUP_YAW_MAX;
    else                                           integralYaw = -WINDUP_YAW_MAX;

    PIDoutputYaw = yaw_kp*errorYaw + yaw_ki*integralYaw + yaw_kd*derivateYaw;
}

/*******************************************************************************
* @brief PID control for Pitch.
*
* @param filter_pointer A pointer to struct containing filter variables.
* 
* @return void
******************************************************************************/
void PID_Pitch(FILTER_complement_struct *filter_pointer)
{
  errorPitch    = desired_pitch_angle - filtered_pitch_angle;
  derivatePitch = 0 - (filter_pointer->gyr_x - GYRO_BIAS_X);
  
  if (abs((int)integralPitch) <= WINDUP_PITCH_MAX) integralPitch += errorPitch*dt;
  else if (integralPitch > 0)                      integralPitch = WINDUP_PITCH_MAX;
  else                                             integralPitch = -WINDUP_PITCH_MAX;
  
  PIDoutputPitch = pitch_kp*errorPitch + pitch_ki*integralPitch + pitch_kd*derivatePitch;
}

/*******************************************************************************
* @brief PID control for Roll.
*
* @param filter_pointer A pointer to struct containing filter variables.
* 
* @return void
******************************************************************************/
void PID_Roll(FILTER_complement_struct *filter_pointer)
{
  errorRoll    = desired_roll_angle - filtered_roll_angle;
  derivateRoll = 0 - (filter_pointer->gyr_y - GYRO_BIAS_Y);
  
  if (abs((int)integralRoll) <= WINDUP_ROLL_MAX) integralRoll += errorRoll*dt;
  else if (integralRoll > 0)                     integralRoll = WINDUP_ROLL_MAX;
  else                                           integralRoll = -WINDUP_ROLL_MAX;
  
  PIDoutputRoll = roll_kp*errorRoll + roll_ki*integralRoll + roll_kd*derivateRoll;
}

/*******************************************************************************
* @brief  Motor limit control and compute pulse-widths for each motor.
* 
* @param  void
* 
* @return void
* 
* @detail Function that sets the regulated values to each motor. 
*         Checks maximun and minimun values for the motors.
******************************************************************************/
void motorControl(void)
{
  /* PID MONSTER */
  RFmotor = ThrustOnMotor + lroundf( PIDoutputRoll - PIDoutputPitch + PIDoutputYaw);
  LFmotor = ThrustOnMotor + lroundf(-PIDoutputRoll - PIDoutputPitch - PIDoutputYaw);
  RBmotor = ThrustOnMotor + lroundf( PIDoutputRoll + PIDoutputPitch - PIDoutputYaw);
  LBmotor = ThrustOnMotor + lroundf(-PIDoutputRoll + PIDoutputPitch + PIDoutputYaw);
  
  /* Max duty cycle check */
  if(RFmotor > 2000)
    RFmotor = 2000;
  if(LFmotor > 2000)
    LFmotor = 2000;
  if(RBmotor > 2000)
    RBmotor = 2000;
  if(LBmotor > 2000)
    LBmotor = 2000;
  
  /* Min duty cycle check */
  if(RFmotor < 1200)
    RFmotor = 1200;
  if(LFmotor < 1200)
    LFmotor = 1200;
  if(RBmotor < 1200)
    RBmotor = 1200;
  if(LBmotor < 1200)
    LBmotor = 1200;
}

/*******************************************************************************
* @brief Change velocity of a motor with a given pulse-width.
*
* @param velocityBasedOnPulseWidth Pulse-width to set the given motor to.
* 
* @param whichMotorToChange The 1-indexed motor index to set the pulse-width for.
*
* @return void
* 
* @detail Takes an input and changes the Pulse Width to input. Then
*         changes to the new value and starts PWM again to selected motor.
******************************************************************************/
void changeVelocityOnMotorsWithPulseWidth (int velocityBasedOnPulseWidth,
                                           int whichMotorToChange)
{
  /* Set up */
  sConfigOC.Pulse = velocityBasedOnPulseWidth;
  
  
  /* Update motor speed */
  switch (whichMotorToChange) 
  {
    
  /* Right forward motor */
  case 1 :
    HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    break;
  
  /* Left  forward motor */  
  case 2 :
    HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    break;
  
  /* Right back motor */
  case 3 :
    HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
    break;
  
  /* Left  back motor */
  case 4 :
    HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
    break;

  /* Default, do nothing */
  default :
    break;
  }
  
  /* Return */
  return;
}