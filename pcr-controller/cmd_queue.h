/*
 *  r3PCR Teensy Controller Code
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

#ifndef CMD_QUEUE_INCLUDE_H_
#define CMD_QUEUE_INCLUDE_H_

#include <avr/io.h>

/* The only actual reason for this queue is to enable non-blocking reads.
*  I.e. we want to avoid blocking the main loop while waiting on a user-input terminating '\n', '\r' or ',' .
* As we could (and DO if the queue is full) execute a command right away once all arguments have been read,
* the queue really does not need to be 4 long and the whole handling could me much much simpler.
* I.e. only ever remember ONE command and execute it as soon as all args are present!
*/

uint8_t cmdq_addCharToArgumentBuffer(uint8_t c);
void cmdq_queueCmdWithNumArgs(void* fptr, uint8_t num_args);
void cmdq_doWork(void);

#endif