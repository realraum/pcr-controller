/*
 *  r3PCR Teensy Controller Code
 *
 *
 *  Copyright (C) 2013 Bernhard Tittelbach <xro@realraum.at>
*   uses avr-utils, anyio & co by Christian Pointner <equinox@spreadspace.org>
 *
 *  This file is part of spreadspace avr utils.
 *
 *  spreadspace avr utils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  spreadspace avr utils is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with spreadspace avr utils. If not, see <http://www.gnu.org/licenses/>.
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <stdio.h>

#include "util.h"
#include "led.h"
#include "anyio.h"

#include "onewire.h"
#include "ds1820.h"

#include "pwm.h"
#include "pid_control.h"
#include "temp_curve.h"
#include "cmd_queue.h"

#define PIN_HIGH(PORT, PIN) PORT |= (1 << PIN)
#define PIN_LOW(PORT, PIN) PORT &= ~(1 << PIN)
#define PINMODE_OUTPUT PIN_HIGH  //just use DDR instead of PORT
#define PINMODE_INPUT PIN_LOW  //just use DDR instead of PORT

#define OP_SETBIT |=
#define OP_CLEARBIT &= ~
#define OP_CHECK &
#define PIN_SW(PORTDDRREG, PIN, OP) PORTDDRREG OP (1 << PIN)

#define HIGHv OP_SETBIT
#define LOWv OP_CLEARBIT

#define PUMP_PIN PINB3
#define PELTIER_INA  PINF7
#define PELTIER_INB  PINB6
#define PELETIER_PWM_EN PINB5
#define TOPHEAT_PIN PIND7

uint8_t num_temp_sensors_ = 0;
int16_t raw_temp_ = 0;
uint8_t debug_ = 0;
uint8_t monitor_temp_ = 0;
// at f_system_clk = 10Hz, system_clk_ will not overrun for at least 13 years. PCR won't run that long
uint32_t system_clk_ = 0;

//with F_CPU = 16MHz and TIMER3 Prescaler set to /1024, TIMER3 increments with f = 16KHz. Thus if TIMER3 reaches 16, 1ms has passed.
#define T3_MS     *16
//set TICK_TIME to 1/10 of a second
#define	TICK_TIME (100 T3_MS)

ISR(TIMER3_COMPA_vect)
{
  //increment system_clk every TIME_TICK (aka 100ms)
	system_clk_++;
  //set up "clock" comparator for next tick
  OCR3A = (OCR3A + TICK_TIME) & 0xFFFF;
}

void initSysClkTimer3(void)
{
  system_clk_ = 0;
  // set counter to 0
  TCNT3 = 0x0000;
	// no outputs
	TCCR3A = 0;
	// Prescaler for Timer3: F_CPU / 1024 -> counts with f= 16KHz ms
	TCCR3B = _BV(CS32) | _BV(CS30);
	// set up "clock" comparator for first tick
	OCR3A = TICK_TIME & 0xFFFF;
	// enable interrupt
	TIMSK3 = _BV(OCIE3A);
}

void queryAndSaveTemperature(uint8_t bit_resolution)
{
    uint8_t sensor_index = 0;

    if (num_temp_sensors_ == 0)
    {
        num_temp_sensors_ = ds1820_discover();
    }

    for (sensor_index=0; sensor_index < num_temp_sensors_; sensor_index++)
    {
        ds1820_set_resolution(sensor_index, bit_resolution);
        ds1820_start_measuring(sensor_index);
    }

    ds1820_wait_conversion_time(bit_resolution);

    for (sensor_index=0; sensor_index < num_temp_sensors_; sensor_index++)
    {
        raw_temp_ = ds1820_read_temperature(sensor_index);
        if (raw_temp_ != DS1820_ERROR)
        {
            break; //we need only one successfully read value
        }
    }
}

void printRawTemp(int16_t raw_temp)
{
  int16_t decimals = 100 * (raw_temp % 16) / 16;
  if (decimals < 0)
    decimals *= -1;
  printf("%d.%02d", raw_temp / 16, decimals);
}

void printStatus(void)
{
  if (num_temp_sensors_ == 0)
  {
    printf("{\"cmd_ok\":false,\"error\": \"No DS1820 sensors on 1wire bus, thus no temperature\"}\r\n");
    return;
  }
  if (raw_temp_ == DS1820_ERROR)
  {
      printf("{\"cmd_ok\":false,\"error\":\"talking to DS18b20, no valid temperature!\"}\r\n");
  } else {
      printf("{\"t\":%lu, \"currtemp\":", system_clk_);
      printRawTemp(raw_temp_);
      printf(", \"targettemp\":");
      printRawTemp(pid_getTargetValue());
      printf(", \"curve\":%s", (tcurve_isSet()?"true":"false"));
      printf(", \"curve_t_elapsed\":%u", tcurve_getTimeElapsed());
      printf(", \"cycles_left\":%u}\r\n", tcurve_getRepeatsLeft());
  }
}

//~ void readIntoBuffer(char *buffer, uint8_t buflen)
//~ {
  //~ while (anyio_bytes_received() == 0);
  //~ int ReceivedByte=0;
  //~ do {
    //~ ReceivedByte = fgetc(stdin);
    //~ if (ReceivedByte != EOF)
    //~ {
      //~ *buffer = (char) ReceivedByte;
      //~ buffer++;
      //~ buflen --;
    //~ }
  //~ } while (ReceivedByte != '\n' && ReceivedByte != '\r' && buflen > 1);
  //~ *buffer = 0;
//~ }

//~ int16_t readNumber(void)
//~ {
  //~ char buffer[20];
  //~ readIntoBuffer(buffer, 20);
  //~ return atoi(buffer);
//~ }

void setPeltierCoolingDirectionPower(int16_t value)
{
  if (value > 255)
    value = 255;
  if (value < -255)
    value = -255;

  if (value >= 0)
  {
    PIN_HIGH(PORTF, PELTIER_INA);
    PIN_LOW(PORTB, PELTIER_INB);
    pwm_set((uint8_t) value);
  } else {
    PIN_LOW(PORTF, PELTIER_INA);
    PIN_HIGH(PORTB, PELTIER_INB);
    pwm_set((uint8_t) (-1 * value));
  }
  if (debug_)
    printf("Peltier value: %d, INA: %d, INB: %d\r\n", value, (PORTF & _BV(PELTIER_INA)) > 0, (PORTB & _BV(PELTIER_INB)) > 0);
}

void handle_cmd(uint8_t cmd)
{
  switch(cmd) {
  case ' ':
  case '\n':
  case '\r':
    return;
  case 'R':
  case 'r': reset2bootloader(); break;
  case '?': debug_ = ~debug_; break;
  case 'm': monitor_temp_ = ~monitor_temp_; break;
  case '=': pid_setTargetValue(raw_temp_); break;
  case '#': pid_setTargetValue(PID_DISABLED); break;
  case 't':
  case 's': printStatus(); return;
  case 'L': led_toggle(); break;
  case 'l': cmdq_queueCmdWithNumArgs(led_toggle, 0); break;
  case 'p':
  case 'i':
  case 'd':
    pid_printVars();
    return;
  case 'T': cmdq_queueCmdWithNumArgs(pid_setTargetValue, 1); break;
  case 'P': cmdq_queueCmdWithNumArgs(pid_setP, 1); break;
  case 'I': cmdq_queueCmdWithNumArgs(pid_setI, 1); break;
  case 'D': cmdq_queueCmdWithNumArgs(pid_setD, 1); break;
  case 'A': PIN_HIGH(PORTB, PUMP_PIN); break;
  case 'a': PIN_LOW(PORTB, PUMP_PIN); break;
  case 'B': PIN_HIGH(PORTD, TOPHEAT_PIN); break;
  case 'b': PIN_LOW(PORTD, TOPHEAT_PIN); break;
  case '-': //reset temp curve
    tcurve_reset();
    break;
  case '+': //add temp curve entry
    //~ tcurve_add(readNumber(), readNumber());
    cmdq_queueCmdWithNumArgs(tcurve_add, 2);
    break;
  case 'Z': cmdq_queueCmdWithNumArgs(tcurve_setRepeats, 1); break;
  case 'E': cmdq_queueCmdWithNumArgs(tcurve_setPostCycleTargetTemp, 1); break;
  default: printf("{\"cmd_ok\":false,\"error\":\"unknown cmd\"}\r\n"); return;
  }
  printf("{\"cmd_ok\":true}\r\n");
}

int main(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  cpu_init();
  led_init();
  anyio_init(115200, 0);
  sei();

  led_off();
  owi_init(PINC7, &PINC);
  PINMODE_OUTPUT(DDRB, PUMP_PIN);
  PIN_LOW(PORTB, PUMP_PIN);
  PINMODE_OUTPUT(DDRB, PELETIER_PWM_EN);
  PINMODE_OUTPUT(DDRB, PELTIER_INB);
  PINMODE_OUTPUT(DDRF, PELTIER_INA);
  PINMODE_OUTPUT(DDRD, TOPHEAT_PIN);

  pwm_init();
  pwm_set(0);

  pid_loadFromEEPROM();

  num_temp_sensors_ = ds1820_discover();

  uint32_t last_time = 0;
  uint32_t last_time2 = 0;
  initSysClkTimer3(); //start system clock

  for(;;)
  {
    int16_t BytesReceived = anyio_bytes_received();
    while(BytesReceived > 0)
    {
      int ReceivedByte = fgetc(stdin);
      if (ReceivedByte != EOF)
      {
        // Ask cmdq_addCharToArgumentBuffer if it wants the current char, otherwise let handle_cmd() have it
        if (cmdq_addCharToArgumentBuffer(ReceivedByte))
          handle_cmd(ReceivedByte);
      }
      BytesReceived--;
    }

    cmdq_doWork();  //may call queued functions

    queryAndSaveTemperature(11); //at 11bit resolution, this takes at least 390ms

    if (monitor_temp_)
      printStatus();

    if (tcurve_isSet())
    {
      uint16_t time_elapsed = (uint16_t) (system_clk_ - last_time);
      last_time = system_clk_;
      //PID_DISABLED == TCURVE_ERROR so this works out fine and we disable heating and PID in this case
      pid_setTargetValue(tcurve_getTempToSet(raw_temp_, time_elapsed));
      if (debug_)
      {
        printf("t: %lu, elapsed: %u, target_temp: %d\r\n", system_clk_, time_elapsed, pid_getTargetValue());
      }
    }

    // PID control
    // make sure this is called at exact periodic intervals (i.e. make sure there are no large variable delays in for loop)
    // e.g. enable periodic temp monitoring 'm' rather than querying temp at some intervall 's'
    if (pid_isEnabled())
    {
      while (system_clk_ - last_time2 < 5 * TICK_TIME); //wait until at least 500ms have passed since last time. Should be enough time for everything else to finish. (after 13 years, code will hang here)
      last_time2 = system_clk_;
      setPeltierCoolingDirectionPower(pid_calc(raw_temp_));
    }
    else
      setPeltierCoolingDirectionPower(0);

    anyio_task();
  }
}
