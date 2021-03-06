/*
 *  pcr-controller
 *
 *
 *  Copyright (C) 2013-2014 Bernhard Tittelbach <xro@realraum.at>
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
 *  along with pcr-controller. If not, see <http://www.gnu.org/licenses/>.
*/

#include "temp_curve.h"
#include <stdlib.h>
#include <stdio.h>

extern uint8_t debug_;

const int16_t temp_margin_ = 16; // 1 °C

typedef struct tc_entry tc_entry;
struct tc_entry {
    int16_t target_temp;
    uint16_t hold_for_timeticks;
    tc_entry *next;
};

tc_entry *temp_curve_ = 0;  //pointer to start of curve (so we can free it by going through the list)
tc_entry *temp_curve_end_ = 0;  //pointer to last added entry in curve (so we know can quickly append new entries)
tc_entry *temp_curve_current_ = 0;  //pointer to currently active entry (temp we currently hold)
tc_entry *temp_curve_loop_first_ = 0;  //pointer to start of loop (loop is a subset of curve list)
tc_entry *temp_curve_loop_last_ = 0;  //pointer to end of loop (loop is a subset of curve list)

uint16_t temp_stable_time_ = 0;

uint8_t temp_curve_original_num_repeats_ = 0;
uint8_t curve_loop_num_repeats_ = 0;   // number of times the loop still needs to be reapeated before finishing the rest of the curve

void tcurve_reset(void)
{
    tc_entry *curr = temp_curve_;
    tc_entry *next = 0;
    while (curr != 0) {
        next = curr->next;
        if (debug_)
            printf("tcreset: curr: 0x%x, next: 0x%x, end: 0x%x\r\n",(uint16_t)curr,(uint16_t)next,(uint16_t)temp_curve_end_);
        free(curr);
        if (curr == temp_curve_end_)
            break; //just to be sure
        curr = next;
    }
    temp_curve_ = 0;
    temp_curve_end_ = 0;
    temp_curve_current_ = 0;
    temp_curve_original_num_repeats_ = 0;
    curve_loop_num_repeats_ = 0;
    temp_curve_loop_first_ = temp_curve_;
    temp_curve_loop_last_ = 0;
    if (debug_)
        printf("tcreset: done\n\n");
}

uint16_t tcurve_getTimeElapsed(void)
{
    return temp_stable_time_;
}

uint8_t tcurve_getRepeatsLeft(void)
{
    return curve_loop_num_repeats_;
}

uint8_t tcurve_isSet(void)
{
    return temp_curve_ != 0;
}

void tcurve_setRepeats(uint8_t r)
{
    curve_loop_num_repeats_ = r;
    temp_curve_original_num_repeats_ = r;
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
        temp_curve_loop_first_ = new_entry;
    } else {
        temp_curve_end_->next = new_entry;
        temp_curve_end_ = new_entry;
    }
}

void tcurve_setRepeatStartPosToLatestEntry(void)
{
    //calling this again, overwrites last set loop start entry
    temp_curve_loop_first_= temp_curve_end_;
    //no loop_last_ before loop_first_
    temp_curve_loop_last_ = 0;
}

void tcurve_setRepeatEndPosToLatestEntry(void)
{
    //calling this again, overwrites last set loop stop entry
    temp_curve_loop_last_ = temp_curve_end_;
}

void tcurve_printCurve(uint8_t cmd)
{
    if (temp_curve_ == 0)
    {
        printf("{\"cmd\":\"%c\",\"cmd_ok\":false,\"error\":\"No curve set\"}\r\n",cmd);
        return;
    }
    printf("{\"cmd\":\"%c\",\"curve\":[",cmd);
    for (tc_entry *ce = temp_curve_; ; ce=ce->next)
    {
        printf("{\"temp\":%d,\"duration\":%u,\"is_curr\":%d,\"is_loop_start\":%d,\"is_loop_end\":%d},",
            ce->target_temp,
            ce->hold_for_timeticks,
            ce == temp_curve_current_,
            ce == temp_curve_loop_first_,
            ce == temp_curve_loop_last_);

        if (ce == temp_curve_end_)
            break;
    }
    printf("0],\"loop_repeats\":%d}\r\n", temp_curve_original_num_repeats_);
}

int16_t tcurve_getTempToSet(int16_t current_temp, uint16_t ticks_elapsed)
{
    if (temp_curve_current_ == 0)
        return TCURVE_ERROR;

    if (current_temp > temp_curve_current_->target_temp - temp_margin_ && current_temp < temp_curve_current_->target_temp + temp_margin_)
    {
        temp_stable_time_ += ticks_elapsed;
    } // else: ignore the case of temp falling temporarily outside of temp_margin_

    //if temp has been stable for the set amount of timeticks in current curve element
    if (temp_stable_time_ >= temp_curve_current_->hold_for_timeticks)
    {
        temp_stable_time_ = 0;

        if (curve_loop_num_repeats_ && //if repetitions left
            ( (temp_curve_loop_last_ != 0 && temp_curve_current_ == temp_curve_loop_last_)    //and we have reached a set loop endpoint
            || temp_curve_current_->next == 0       //or the end of the list
            ))
        {   //go back to first point in loop
            curve_loop_num_repeats_--;
            temp_curve_current_ = temp_curve_loop_first_;
        } else if (temp_curve_current_->next != 0)  //else if elements left in curve list
        {   // go to next element
            temp_curve_current_ = temp_curve_current_->next;
        }
        //else stay on temperature of last element forever (until reset is called or new elements are added)
    }
    return temp_curve_current_->target_temp;
}
