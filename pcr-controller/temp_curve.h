/*
 *  rPCR Teensy Controller Code
 *
 *
 *  Copyright (C) 2013 Bernhard Tittelbach <xro@realraum.at>
 *
 *  This code is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  This code is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with spreadspace avr utils. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TEMP_CURVE_INCLUDE_H_
#define TEMP_CURVE_INCLUDE_H_

#define TCURVE_ERROR (1<<15)

#include <avr/io.h>

void tcurve_reset(void);
void tcurve_setRepeats(uint8_t r);
void tcurve_setRepeatStartPosToLatestEntry(void);
uint8_t tcurve_isSet(void);
uint8_t tcurve_hasFinished(void);
uint16_t tcurve_getTimeElapsed(void);
uint8_t tcurve_getRepeatsLeft(void);
void tcurve_setPostCycleTargetTemp(int16_t v);
void tcurve_add(int16_t temp, uint16_t hold_for_s);
int16_t tcurve_getTempToSet(int16_t current_temp, uint16_t time_elapsed);
void tcurve_printCurve(void);

#endif