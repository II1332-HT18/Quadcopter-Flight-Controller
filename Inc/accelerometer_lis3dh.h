/** accelerometer_lis3dh.h */

#ifndef __ACCELEROMETER_LIS3DH_H
#define __ACCELEROMETER_LIS3DH_H

/*enum acc_state {
  ACC_FREE = 0;
  ACC_BLOCKED = 1;
};*/

/* INCLUDES *******************************************************************/
#include "stm32f3xx_hal.h"

/* DEFINES ********************************************************************/


/* TYPEDEF ********************************************************************/
typedef struct {
  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;
  
  int16_t pitch;
  int16_t roll;
  
} ACC_TypeDef;

/* GLOBAL FUNCTIONS ***********************************************************/
void ACC_update_xyz();
void ACC_get_address(ACC_TypeDef** acc_values_ptr);
void ACC_get_struct(ACC_TypeDef* pointer);
void ACC_get_pitch_angle(ACC_TypeDef* values);
void ACC_get_roll_angle(ACC_TypeDef* values);

#endif