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

#include "temp_curve.h"
#include <stdlib.h>

const int16_t temp_margin_ = 8; // 0.5 °C

typedef struct tc_entry tc_entry;
struct tc_entry {
    int16_t target_temp;
    uint16_t hold_for_timeticks;
    tc_entry *next;
};

tc_entry *temp_curve_ = 0;
tc_entry *temp_curve_end_ = 0;
tc_entry *temp_curve_current_ = 0;

uint16_t temp_stable_time_ = 0;

uint8_t curve_num_repeats_ = 0;

void tcurve_reset(void)
{
    tc_entry *curr = temp_curve_;
    tc_entry *next = 0;
    while (curr != 0) {
        next = temp_curve_->next;
        free(curr);
        curr = next;
    }
    temp_curve_ = 0;
    temp_curve_end_ = 0;
    temp_curve_current_ = 0;
    curve_num_repeats_ = 0;
}

uint8_t tcurve_hasFinished(void)
{
    return temp_curve_current_ == temp_curve_end_ && curve_num_repeats_ == 0;
}

uint8_t tcurve_isSet(void)
{
    return temp_curve_ != 0;
}

void tcurve_setRepeats(uint8_t r)
{
    curve_num_repeats_ = r;
}

void tcurve_add(int16_t temp, uint16_t hold_for_ticks)
{
    tc_entry *new_entry = malloc(sizeof(tc_entry));
    new_entry->target_temp = temp;
    new_entry->hold_for_timeticks = hold_for_ticks;
    new_entry->next = 0;

    if (temp_curve_end_ == 0)
    {
        temp_curve_end_ = new_entry;
        temp_curve_ = new_entry;
        temp_curve_current_ = new_entry;
    } else {
        temp_curve_end_->next = new_entry;
        temp_curve_end_ = new_entry;
    }
}

int16_t tcurve_getTempToSet(int16_t current_temp, uint16_t ticks_elapsed)
{
    if (temp_curve_current_ == 0)
        return TCURVE_ERROR;

    if (current_temp > temp_curve_current_->target_temp - temp_margin_ && current_temp < temp_curve_current_->target_temp + temp_margin_)
    {
        temp_stable_time_ += ticks_elapsed;
    } // else: ignore the case of temp falling temporarily outside of temp_margin_

    // if time has been stable for long enough, advance to next
    // if there is no next, repeat curve until curve_num_repeats_ == 0
    // once that happens, hold current temp forever
    if (temp_stable_time_ >= temp_curve_current_->hold_for_timeticks)
    {
        if (temp_curve_current_->next != 0)
        {
            temp_curve_current_ = temp_curve_current_->next;
            temp_stable_time_ = 0;
        } else if (curve_num_repeats_)
        {
            //restart temp curve from the beginning
            temp_curve_current_ = temp_curve_;
            curve_num_repeats_--;
        }
    }
    return temp_curve_current_->target_temp;
}