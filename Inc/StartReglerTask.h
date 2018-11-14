
#ifndef __STARTREGLERTASK_H
#define __STARTREGLERTASK_H



#include "main.h"


void StartReglerTask(void const * arguments);

extern osMailQId sensor_mailbox;
extern osMailQId filter_mailbox;

#endif