
/*
Mailbox communication from PWMIN
*/

#ifndef __PWMIN_H
#define __PWMIN_H

#include "main.h"


/* Mailboxes */
extern osMailQId pwmIn_mailbox;


void StartPwmInTask(void const * argument);

#endif