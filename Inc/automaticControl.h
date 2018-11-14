/*******************************automaticControl.h*****************************/

#ifndef __AUTOMATICCONTROL_H
#define __AUTOMATICCONTROL_H

/* Function prototypes -------------------------------------------------------*/
int abs(int i);

/* Private function prototypes -----------------------------------------------*/
void automaticControl(main_struct*);

//void pwmOut(main_struct*);  //Not implemeted
void motorControl(void);
void changeVelocityOnMotorsWithPulseWidth (int velocityBasedOnPulseWidth,
                                           int whichMotorToChange);

/*PID functions*/
void PID_Pitch(FILTER_complement_struct *filter_pointer);
void PID_Roll(FILTER_complement_struct *filter_pointer);
void PID_Yaw(FILTER_complement_struct *filter_pointer);

void emergencyStop(void);

#endif /* AUTOMATICCONTROL_H */

