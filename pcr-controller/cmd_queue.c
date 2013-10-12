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

#include <stdlib.h>
#include "cmd_queue.h"


typedef struct {
    uint8_t read_num_numbers;
    void* function_ptr;
} cmd_queue_item;

#define CMQ_QUEUE_LENGTH 4
cmd_queue_item cmd_queue_[CMQ_QUEUE_LENGTH];
uint8_t cmd_queue_todo_pos_ = 0;
uint8_t cmd_queue_next_here_ = 0;

#define CMDQ_INPUT_LIST_LEN  2
int16_t input_list_[CMDQ_INPUT_LIST_LEN];
uint8_t num_args_read_ = 0;

#define CMDQ_READBUFFER_LEN  20
char cmdq_readbuffer_[CMDQ_READBUFFER_LEN];
char *cmdq_readbuffer_pos_ = cmdq_readbuffer_;
char *const cmdq_readbuffer_end_ = cmdq_readbuffer_ + (CMDQ_READBUFFER_LEN - 1);

inline void cmdq_finishReadingArgument(void)
{
    *cmdq_readbuffer_pos_ = '\0';
    cmdq_readbuffer_pos_ = cmdq_readbuffer_;
    input_list_[num_args_read_++] = atoi(cmdq_readbuffer_);
}

//call regularily if in state CQ_READING
uint8_t cmdq_addCharToArgumentBuffer(uint8_t c)
{
    if ( cmd_queue_todo_pos_ == cmd_queue_next_here_ )
        return 1; //nothing to do with us
    if (num_args_read_ >= CMDQ_INPUT_LIST_LEN)
        return 1;  //nothing to do with us
    if (num_args_read_ >= cmd_queue_[cmd_queue_todo_pos_].read_num_numbers )
        return 1;  //nothing to do with us

    //if input terminated by \n or \r then addArgument
    if (c == '\n' || c == '\r')
    {
        cmdq_finishReadingArgument();
        return 0;
    } else {    //continue reading
        *cmdq_readbuffer_pos_ = (char) c;
        cmdq_readbuffer_pos_++;
    }
    //if numlen of readbuffer reached, addArgument as well
    if (cmdq_readbuffer_pos_ == cmdq_readbuffer_end_)
    {
        cmdq_finishReadingArgument();
        return 0;
    }
    return 0;
}

void cmdq_queueCmdWithNumArgs(void* fptr, uint8_t num_args)
{
    if (num_args > CMDQ_INPUT_LIST_LEN)
        return; //can't do that Would hang cmdq
    cmd_queue_[cmd_queue_next_here_].read_num_numbers=num_args;
    cmd_queue_[cmd_queue_next_here_].function_ptr=fptr;
    cmd_queue_next_here_++;
    cmd_queue_next_here_ %= CMQ_QUEUE_LENGTH;
}

void cmdq_doWork(void)
{
    if ( cmd_queue_todo_pos_ == cmd_queue_next_here_ )
        return;

    // is num_args_read_ now equal to num args we expect ?
    if (num_args_read_ == cmd_queue_[cmd_queue_todo_pos_].read_num_numbers )
    {
        switch (num_args_read_)
        {
            case 0:
                ((void(*)(void)) cmd_queue_[cmd_queue_todo_pos_].function_ptr)();
                break;
            case 1:
                ((void(*)(int16_t)) cmd_queue_[cmd_queue_todo_pos_].function_ptr)(input_list_[0]);
                break;
            case 2:
                ((void(*)(int16_t,int16_t)) cmd_queue_[cmd_queue_todo_pos_].function_ptr)(input_list_[0], input_list_[1]);
                break;
        }
        num_args_read_ = 0;
        cmd_queue_todo_pos_++;
        cmd_queue_todo_pos_ %= CMQ_QUEUE_LENGTH;
    }
}