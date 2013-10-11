/*
 *  OpenPCR Teensy Controller Code
 *
 *
 *  Copyright (C) 2013 Bernhard Tittelbach <xro@realraum.at>
*/

#ifndef PID_CONTROLLER_INCLUDE_H_
#define PID_CONTROLLER_INCLUDE_H_

#include <stdio.h>

#define PID_DISABLED (1<<15)

void pid_setP(int16_t p);
void pid_setI(int16_t i);
void pid_setD(int16_t d);
void pid_printVars(void);
void pid_setTargetValue(int16_t v);
int pid_isEnabled(void);
int16_t pid_getTargetValue(void);
int16_t pid_calc(int16_t current_value);
void pid_loadFromEEPROM(void);
void pid_saveToEEPROM(void);

#endif
