/*
Mailbox communication from PWMIN
*/

#ifndef __ANALYS_H
#define __ANALYS_H

#include "main.h"
#include <stdio.h>
#include <string.h>


/* Mailboxes */
extern osMailQId analys_mailbox;

/* Function prototype --------------------------------------------------------*/
void StartAnalysTask(void const * argument);
void getValuesFromStruct(void);
void addValuesToBuffer(void);

#endif