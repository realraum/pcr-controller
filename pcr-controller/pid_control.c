/*
 *  OpenPCR Teensy Controller Code
 *
 *
 *  Copyright (C) 2013 Bernhard Tittelbach <xro@realraum.at>
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
int32_t pid_d_last_error_ = 0;

uint16_t pid_target_value_ = 0;

void pid_setP(int16_t p)
{
    pid_p_ = (int32_t) p * PID_SCALE;
    pid_saveToEEPROM();
}

void pid_setI(int16_t i)
{
    pid_i_ = (int32_t) i * PID_SCALE;
    pid_saveToEEPROM();
}

void pid_setD(int16_t d)
{
    pid_d_ = (int32_t) d * PID_SCALE;
    pid_saveToEEPROM();
}

void pid_printVars(void)
{
    printf("PID P: %d\r\nPID I: %d\r\nPID D: %d\r\n", (int16_t) (pid_p_ / PID_SCALE), (int16_t) (pid_i_ / PID_SCALE), (int16_t) (pid_d_ / PID_SCALE));
}

void pid_setTargetValue(uint16_t v)
{
    pid_target_value_ = v; 
}

uint16_t pid_getTargetValue(void)
{
    return pid_target_value_;
}

// PRE-CONDITION: call pid_calc at exactly regular intervals !!
int16_t pid_calc(uint16_t current_value)
{
    int32_t		error = pid_target_value_ - current_value;
    // derivative
    // instead of derivative of error we take derivative on measurement
    // since dError/dt = - dInput/dt   (note the - )    
    int32_t		d_measure = current_value - pid_d_last_error_;
    pid_d_last_error_ = error;

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