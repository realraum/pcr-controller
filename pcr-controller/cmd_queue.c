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

#include <stdlib.h>
#include <stdio.h>
#include "cmd_queue.h"

//static assert:
//~ static int SDFLKJ[sizeof(int) == sizeof(int16_t)] = { 0 };

#define CMDQ_INPUT_LIST_LEN  2

typedef struct {
    uint8_t num_args;
    uint8_t cmd;
    void* function_ptr;
    int16_t input_list[CMDQ_INPUT_LIST_LEN];
    uint8_t num_args_read;
} cmd_queue_item;

#define CMQ_QUEUE_LENGTH 4
cmd_queue_item cmd_queue_[CMQ_QUEUE_LENGTH];
uint8_t cmd_queue_todo_pos_ = 0;
uint8_t cmd_queue_fillargs_pos_ = 0;
uint8_t cmd_queue_next_here_ = 0;



#define CMDQ_READBUFFER_LEN  20
char cmdq_readbuffer_[CMDQ_READBUFFER_LEN];
uint8_t cmdq_readbuffer_pos_ = 0;

inline void cmdq_finishReadingArgument(void)
{
    cmdq_readbuffer_[cmdq_readbuffer_pos_] = '\0';
    cmdq_readbuffer_pos_ = 0;
    cmd_queue_[cmd_queue_fillargs_pos_].input_list[cmd_queue_[cmd_queue_fillargs_pos_].num_args_read++] = atoi(cmdq_readbuffer_);
}

//call regularily if in state CQ_READING
uint8_t cmdq_addCharToArgumentBuffer(uint8_t c)
{
    if ( cmd_queue_todo_pos_ == cmd_queue_next_here_ )
        return 1; //nothing to do with us, since no cmd's queued
    if (cmd_queue_[cmd_queue_fillargs_pos_].num_args_read >= CMDQ_INPUT_LIST_LEN)
        return 1;  //nothing to do with us
    if (cmd_queue_[cmd_queue_fillargs_pos_].num_args_read >= cmd_queue_[cmd_queue_fillargs_pos_].num_args )
        return 1;  //nothing to do with us
    //if input terminated by \n or \r then addArgument
    if (c == '\n' || c == '\r'  || c == ',' || c == 0x1B)
    {
        cmdq_finishReadingArgument();
        return 0;
    } else {    //continue reading
        cmdq_readbuffer_[cmdq_readbuffer_pos_++] = (char) c;
    }
    //if numlen +1 of readbuffer reached, addArgument as well (leave one char free to terminate with \0)
    if (cmdq_readbuffer_pos_ +1 >= CMDQ_READBUFFER_LEN)
    {
        cmdq_finishReadingArgument();
    }
    return 0;
}

void cmdq_queueCmdWithNumArgs(void* fptr, uint8_t num_args, uint8_t cmd)
{
    if (num_args > CMDQ_INPUT_LIST_LEN)
    {
        printf("{\"cmd\":\"%c\",\"cmd_ok\":false,\"error\":\"max args == %d\"}\r\n", cmd, CMDQ_INPUT_LIST_LEN);
        return; //to continue would hang cmdq, so we don't
    }
    if ((cmd_queue_next_here_ +1) % CMQ_QUEUE_LENGTH == cmd_queue_todo_pos_) //for this check REQUIRED: CMQ_QUEUE_LENGTH > 2
        cmdq_doWork(); //no more space in queue -> execute now !
    cmd_queue_[cmd_queue_next_here_].num_args = num_args;
    cmd_queue_[cmd_queue_next_here_].cmd = cmd;
    cmd_queue_[cmd_queue_next_here_].function_ptr = fptr;
    cmd_queue_[cmd_queue_next_here_].num_args_read = 0;
    cmd_queue_fillargs_pos_ = cmd_queue_next_here_;
    cmd_queue_next_here_++;
    cmd_queue_next_here_ %= CMQ_QUEUE_LENGTH;
}

void cmdq_doWork(void)
{
    while ( cmd_queue_todo_pos_ != cmd_queue_next_here_ )
    {
        // is num_args_read_ now equal to num args we expect ?
        if (cmd_queue_[cmd_queue_todo_pos_].num_args_read != cmd_queue_[cmd_queue_todo_pos_].num_args )
            break;
        if (cmd_queue_[cmd_queue_todo_pos_].function_ptr == 0 )
            break;

        switch (cmd_queue_[cmd_queue_todo_pos_].num_args)
        {
            case 0:
                ((void(*)(void)) cmd_queue_[cmd_queue_todo_pos_].function_ptr)();
                break;
            case 1:
                ((void(*)(int16_t)) cmd_queue_[cmd_queue_todo_pos_].function_ptr)(cmd_queue_[cmd_queue_todo_pos_].input_list[0]);
                break;
            case 2:
                ((void(*)(int16_t,int16_t)) cmd_queue_[cmd_queue_todo_pos_].function_ptr)(cmd_queue_[cmd_queue_todo_pos_].input_list[0], cmd_queue_[cmd_queue_todo_pos_].input_list[1]);
                break;
        }
        char cmd = cmd_queue_[cmd_queue_todo_pos_].cmd;
        cmd_queue_[cmd_queue_todo_pos_].num_args_read = 0;
        cmd_queue_[cmd_queue_todo_pos_].num_args = 0;
        cmd_queue_[cmd_queue_todo_pos_].function_ptr = 0;
        cmd_queue_[cmd_queue_todo_pos_].cmd = 0;
        cmd_queue_todo_pos_++;
        cmd_queue_todo_pos_ %= CMQ_QUEUE_LENGTH;
        printf("{\"cmd\":\"%c\",\"cmd_ok\":true}\r\n",cmd);
    }
}
