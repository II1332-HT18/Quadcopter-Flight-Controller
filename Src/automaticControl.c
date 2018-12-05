/**
******************************************************************************
* @file    automaticControl.c
* @author  Michael Fransson, Michael Henriksson, Ali Qhorbani, Daniel Hooshidar
* @date    15 December 2017
* @brief   This file contains the PID control system that stabilizes flight
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Filter.h"
#include "moving_average.h"

/* External variables --------------------------------------------------------*/
extern TIM_OC_InitTypeDef sConfigOC;
extern TIM_HandleTypeDef htim2;
//extern float dt;

/* Private variables ---------------------------------------------------------*/

static float dt = 0.004;

/* PID variables ----------------*/

/* PID moving averages */
static int RollMovingAverageSampleSize = 5;
static int PitchMovingAverageSampleSize = 5;
static int YawMovingAverageSampleSize = 5;

static MovingAverage *PIDOutputRollAverage;
static MovingAverage *PIDOutputPitchAverage;
static MovingAverage *PIDOutputYawAverage;

/* Roll */
static float roll_kp = 7; //10 
static float roll_ki = 0.3; //0.2
static float roll_kd = 0.54;  //0.4
static float filtered_roll_angle;
static float desired_roll_angle = 0;
static float errorRoll = 0;
static float derivateRoll = 0;
static float integralRoll = 0;
static float PIDoutputRoll = 0;

/* Pitch */
static float pitch_kp = 7;
static float pitch_ki = 0.3;
static float pitch_kd = 0.54;
static float desired_pitch_angle=0;
static float filt_pitch_angle;
static float errorPitch=0;
static float derivatePitch=0;
static float integralPitch=0;
static float PIDoutputPitch=0;

/* Yaw */
static float errorGyroYaw = 0;
static float kp = 0;
static float ki = 0;
static float kd = 0;
static float filtered_yaw_angle;
static float desired_yaw_angle = 0;
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
	PIDOutputRollAverage = moving_average_allocate(RollMovingAverageSampleSize);
	PIDOutputPitchAverage = moving_average_allocate(PitchMovingAverageSampleSize);
	PIDOutputYawAverage = moving_average_allocate(YawMovingAverageSampleSize);

	moving_average_init(PIDOutputRollAverage);
	moving_average_init(PIDOutputPitchAverage);
	moving_average_init(PIDOutputYawAverage);
}

/** ****************************************************************************
 * @brief main function for PID controller 
 *
 * @param pointer to struct containing all global variables
 * 
 ******************************************************************************/
void automaticControl(main_struct* all_values)
{
  /* Get mail */
  osEvent check_mail = osMailGet(pwmIn_mailbox, osWaitForever);
  pwmIn_struct *pwm_pointer = (pwmIn_struct*)check_mail.value.p;
  
  check_mail = osMailGet(sensorFilter_mailbox, osWaitForever);
  FILTER_complement_struct *filter_pointer = (FILTER_complement_struct*)check_mail.value.p;
  
    /* Actual value for Yaw, Pitch and Roll (angle rates from sensor) */
  filtered_yaw_angle = filter_pointer->filter_yaw; // -2.3 due to error from sensor????
  filt_pitch_angle = filter_pointer->filter_pitch;
  filtered_roll_angle = filter_pointer->filter_roll;
  
  /* Setpoint for Yaw, Pitch and Roll */
  desired_yaw_angle = pwm_pointer->yaw;
  desired_pitch_angle = pwm_pointer->pitch - 2; // -2 due to error from controller
  desired_roll_angle = pwm_pointer->roll;

  

  

  /*Run PID algoritm for Yaw, Pitch and Roll*/
  PID_Yaw(filter_pointer);
  PID_Pitch(filter_pointer);
  PID_Roll(filter_pointer);
  
  /* PID moving average calculations */
//  moving_average_move(PIDOutputYawAverage, PIDoutputYaw);
//  moving_average_move(PIDOutputPitchAverage, PIDoutputPitch);
//  moving_average_move(PIDOutputRollAverage, PIDoutputRoll);

//  PIDoutputYaw = moving_average_get_average_float(PIDOutputYawAverage);
//  PIDoutputPitch = moving_average_get_average_float(PIDOutputPitchAverage);
//  PIDoutputRoll = moving_average_get_average_float(PIDOutputRollAverage);
  
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

/** ****************************************************************************
 * @brief Emergency stop to turn off motors
 *
 * @param none
 * 
 * @detail If controller is in emergency stop mode, or thrust is at 0z then bypass PID.
            Sets motor values to 1000 and disables PID calculations.
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

/** ****************************************************************************
 * @brief PID control for Yaw
 *
 * @param pointer to struct containing filter variables.
 * 
 ******************************************************************************/

void PID_Yaw(FILTER_complement_struct *filter_pointer)
{
  /*Yaw control*/
  errorGyroYaw = desired_yaw_angle - filtered_yaw_angle; //Calculate error
  integralYaw += (errorGyroYaw*dt); //I-term
  derivateYaw = filter_pointer->gyr_z;  //D-term
  PIDoutputYaw = (kp*errorGyroYaw + ki*integralYaw + kd*derivateYaw);
}

/** ****************************************************************************
 * @brief PID control for Pitch
 *
 * @param pointer to struct containing filter variables.
 * 
 * @return none
 ******************************************************************************/
void PID_Pitch(FILTER_complement_struct *filter_pointer)
{
  /* Pitch control */  
  
  if(desired_pitch_angle > 25)
    desired_pitch_angle = 25;
  else if(desired_pitch_angle < -25)
    desired_pitch_angle = -25;
  
  /* Pitch control */  
  errorPitch = desired_pitch_angle - filt_pitch_angle - 2; //error
  
  //Windup control
  if(abs((int)integralPitch) <= 50){
    integralPitch += errorPitch*dt/2; //I-term
  }
  else if (integralPitch > 0){
    integralPitch = 50;
  }
  else {
    integralPitch = -50;
  }
  
  derivatePitch = filter_pointer->gyr_x; //D-term
  PIDoutputPitch = (pitch_kp*errorPitch + pitch_ki*integralPitch + pitch_kd*derivatePitch)/6; //control signal
}

/** ****************************************************************************
 * @brief PID control for Roll
 *
 * @param pointer to struct containing filter variables.
 * 
 * @return none
 ******************************************************************************/
void PID_Roll(FILTER_complement_struct *filter_pointer)
{
  /* Roll control */ 
  if(desired_roll_angle > 25)
    desired_roll_angle = 25;
  else if(desired_roll_angle < -25)
    desired_roll_angle = -25;
  
  errorRoll = desired_roll_angle - filtered_roll_angle; //Calculate error
  
  //Windup control
  if(abs((int)integralRoll) <= 50)
      integralRoll += errorRoll*dt/2; //I-term
  
  else if (integralRoll > 0)
      integralRoll = 50;
  
  else
      integralRoll = -50;
  
  derivateRoll = filter_pointer->gyr_y; //D-term
  PIDoutputRoll =(roll_kp*errorRoll + roll_ki*integralRoll + roll_kd*derivateRoll)/6; //control signal
}

/** ****************************************************************************
 * @brief Motor control
 *
 * @param none
 *
 * @detail Function that sets the regulated values to each motor. 
 *         Checks maximun and minimun values for the motors.
 * 
 * @return none
 ******************************************************************************/
void motorControl(void)
{
  /* PID MONSTER */
  RFmotor = (int)(PIDoutputYaw + (int) (ThrustOnMotor + (int)PIDoutputRoll - (int)PIDoutputPitch));
  LFmotor = (int)(-PIDoutputYaw + (int) (ThrustOnMotor - (int)PIDoutputRoll - (int)PIDoutputPitch));
  RBmotor = (int)(-PIDoutputYaw + (int) (ThrustOnMotor + (int)PIDoutputRoll + (int)PIDoutputPitch));
  LBmotor = (int)(PIDoutputYaw + (int) (ThrustOnMotor - (int)PIDoutputRoll + (int)PIDoutputPitch));
  
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

/** ****************************************************************************
 * @brief Change Velocity On Motors With Pulse Width
 *
 * @param PulseWidth Value that changes the speed of the rotor 
 *        Integer that selects which of the four motors to control 
 *
 * @detail Takes an input and changes the Pulse Width to input. Then
 *         changes to the new value and starts PWM again to selected motor.
 * 
 * @return none
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