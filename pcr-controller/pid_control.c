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

#include "pid_control.h"

#include <avr/eeprom.h>

EEMEM int32_t pid_p_eeprom_;
EEMEM int32_t pid_i_eeprom_;
EEMEM int32_t pid_d_eeprom_;
EEMEM int32_t crc_eeprom_;

#define PID_SCALE 1024L

int32_t pid_outlimit_min_ = -255 * PID_SCALE;
int32_t pid_outlimit_max_ = 255 * PID_SCALE;

int32_t pid_p_ = 8192;
int32_t pid_i_ = 512;
int32_t pid_d_ = 24576;

int32_t pid_i_integralsum_ = 0;
int32_t pid_last_input_ = 0;

int16_t pid_target_value_ = PID_DISABLED;

void pid_setP(int16_t p)
{
    pid_p_ = (int32_t) p;
    pid_saveToEEPROM();
}

void pid_setI(int16_t i)
{
    pid_i_ = (int32_t) i;
    pid_saveToEEPROM();
}

void pid_setD(int16_t d)
{
    pid_d_ = (int32_t) d;
    pid_saveToEEPROM();
}

void pid_printVars(void)
{
    printf("PID P: %d /%d\r\nPID I: %d /%d\r\nPID D: %d /%d\r\n", (int16_t) (pid_p_), (int16_t) PID_SCALE, (int16_t) (pid_i_), (int16_t) PID_SCALE, (int16_t) (pid_d_), (int16_t) PID_SCALE);
}

void pid_setTargetValue(int16_t v)
{
    pid_target_value_ = v;
}

int16_t pid_getTargetValue(void)
{
    return pid_target_value_;
}

int pid_isEnabled(void)
{
    return pid_target_value_ != PID_DISABLED;
}

// PRE-CONDITION: call pid_calc at exactly regular intervals !!
int16_t pid_calc(int16_t current_value)
{
    if (pid_target_value_ == PID_DISABLED)
        return PID_DISABLED;

    int32_t		error = pid_target_value_ - current_value;
    // derivative
    // instead of derivative of error we take derivative on measurement
    // since dError/dt = - dInput/dt   (note the - )
    int32_t		d_measure = current_value - pid_last_input_;
    pid_last_input_ = current_value;

    // integral (bring pid_i_ into integral, so we get smooth transfer if pid_i_ suddenly changes)
    pid_i_integralsum_ += pid_i_ * error;

    // prevent integrator wind-up
    if (pid_i_integralsum_ > pid_outlimit_max_)
        pid_i_integralsum_ = pid_outlimit_max_;
    else if (pid_i_integralsum_ < pid_outlimit_min_)
        pid_i_integralsum_ = pid_outlimit_min_;

    // combine factors (insofar as we did not already do it above)
    int32_t pid_output_preclamp = (
        (error * pid_p_) + (pid_i_integralsum_) - (d_measure * pid_d_)
    );

    // limit output
    if (pid_output_preclamp > pid_outlimit_max_)
        return (int16_t) (pid_outlimit_max_ / PID_SCALE);
    else if (pid_output_preclamp < pid_outlimit_min_)
        return (int16_t) (pid_outlimit_min_  / PID_SCALE);
    else
        return (int16_t) (pid_output_preclamp  / PID_SCALE);
}

void pid_loadFromEEPROM(void)
{
    int32_t p = eeprom_read_dword((uint32_t *) &pid_p_eeprom_);
    int32_t i = eeprom_read_dword((uint32_t *) &pid_i_eeprom_);
    int32_t d = eeprom_read_dword((uint32_t *) &pid_d_eeprom_);
    int32_t crc = eeprom_read_dword((uint32_t *) &crc_eeprom_);
    if (crc == (p ^ i ^ d)) {
        pid_p_ = p;
        pid_i_ = i;
        pid_d_ = d;
    }
}

void pid_saveToEEPROM(void)
{
    eeprom_write_dword((uint32_t *) &pid_p_eeprom_, pid_p_);
    eeprom_write_dword((uint32_t *) &pid_i_eeprom_, pid_i_);
    eeprom_write_dword((uint32_t *) &pid_d_eeprom_, pid_d_);
    eeprom_write_word((uint16_t *) &crc_eeprom_,  pid_p_ ^ pid_i_ ^ pid_d_);
}