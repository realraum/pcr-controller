/*
 *  OpenPCR Teensy Controller Code
 *
 *
 *  Copyright (C) 2013 Bernhard Tittelbach <xro@realraum.at>
*/

#ifndef PID_CONTROLLER_INCLUDE_H_
#define PID_CONTROLLER_INCLUDE_H_

#include <stdio.h>

void pid_setP(int16_t p);
void pid_setI(int16_t i);
void pid_setD(int16_t d);
void pid_printVars(void);
void pid_setTargetValue(uint16_t v);
uint16_t pid_getTargetValue(void);
int16_t pid_calc(uint16_t current_value);
void pid_loadFromEEPROM(void);
void pid_saveToEEPROM(void);

#endif
