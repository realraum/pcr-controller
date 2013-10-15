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
#include <stdio.h>

extern uint8_t debug_;

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
tc_entry *temp_curve_restart_pos_ = 0;

uint16_t temp_stable_time_ = 0;

uint8_t curve_num_repeats_ = 0;
uint8_t temp_curve_finished_ = 0;
int16_t post_cycle_target_temp_ = TCURVE_ERROR;

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
    curve_num_repeats_ = 0;
    temp_curve_restart_pos_ = temp_curve_;
    temp_curve_finished_ = 0;
    if (debug_)
        printf("tcreset: done\n\n");
}

uint8_t tcurve_hasFinished(void)
{
    return temp_curve_finished_;
}

uint16_t tcurve_getTimeElapsed(void)
{
    return temp_stable_time_;
}

uint8_t tcurve_getRepeatsLeft(void)
{
    return curve_num_repeats_;
}

uint8_t tcurve_isSet(void)
{
    return temp_curve_ != 0;
}

void tcurve_setRepeats(uint8_t r)
{
    curve_num_repeats_ = r;
    temp_curve_finished_ = 0;
}

void tcurve_setPostCycleTargetTemp(int16_t v)
{
    post_cycle_target_temp_ = v;
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
        temp_curve_restart_pos_ = new_entry;
    } else {
        temp_curve_end_->next = new_entry;
        temp_curve_end_ = new_entry;
    }
}

void tcurve_setRepeatStartPosToLatestEntry(void)
{
   temp_curve_restart_pos_= temp_curve_end_;
}

void tcurve_printCurve(void)
{
    if (temp_curve_ == 0)
    {
        printf("{\"cmd_ok\":false,\"error\":\"No curve set\"}\r\n");
        return;
    }
    printf("{\"curve\":[");
    for (tc_entry *ce = temp_curve_; ; ce=ce->next)
    {
        printf("{\"temp\":%d,\"duration\":%u,\"is_curr\":%d,\"is_loop_start\":%d},",ce->target_temp, ce->hold_for_timeticks, ce == temp_curve_current_,ce == temp_curve_restart_pos_);
        if (ce == temp_curve_end_)
            break;
    }
    printf("0],\"end_temp:\":%d}\r\n", post_cycle_target_temp_);
}

int16_t tcurve_getTempToSet(int16_t current_temp, uint16_t ticks_elapsed)
{
    if (temp_curve_current_ == 0)
        return TCURVE_ERROR;

    if (temp_curve_finished_ && post_cycle_target_temp_ != TCURVE_ERROR)
        return post_cycle_target_temp_;

    if (current_temp > temp_curve_current_->target_temp - temp_margin_ && current_temp < temp_curve_current_->target_temp + temp_margin_)
    {
        temp_stable_time_ += ticks_elapsed;
    } // else: ignore the case of temp falling temporarily outside of temp_margin_

    // if time has been stable for long enough, advance to next
    // if there is no next, repeat curve until curve_num_repeats_ == 0
    // once that happens, set to post_cycle_target_temp_ or hold last value
    if (temp_stable_time_ >= temp_curve_current_->hold_for_timeticks)
    {
        temp_stable_time_ = 0;
        if (temp_curve_current_->next != 0)
        {
            temp_curve_current_ = temp_curve_current_->next;
        } else if (curve_num_repeats_)
        {
            //restart temp curve from the beginning (or the set position)
            temp_curve_current_ = temp_curve_restart_pos_;
            curve_num_repeats_--;
        } else
            temp_curve_finished_ = 1;
    }
    return temp_curve_current_->target_temp;
}