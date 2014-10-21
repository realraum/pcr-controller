/*
 *  r3PCR Teensy Controller Code
 *
 *
 *  Copyright (C) 2013-2014 Bernhard Tittelbach <xro@realraum.at>
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

#define PELTIER_INA_PIN  PINF7
#define PELTIER_INA_PORT  PORTF
#define PELTIER_INA_DDR  DDRF

#define PELTIER_INB_PIN  PINB6
#define PELTIER_INB_PORT  PORTB
#define PELTIER_INB_DDR  DDRB

//OC1A
#define PELETIER_PWM_EN_PIN PINB5
#define PELETIER_PWM_EN_PORT PORTB
#define PELETIER_PWM_EN_DDR DDRB

//OC4D
#define TOPHEAT_PIN PIND7
#define TOPHEAT_PORT PORTD
#define TOPHEAT_DDR DDRD

#define PUMP_PIN PINC6
#define PUMP_PORT PORTC
#define PUMP_DDR DDRC

#define ONEWIRE_PIN PINC7
#define ONEWIRE_PINBASE PINC

uint8_t num_temp_sensors_ = 0;
int16_t raw_temp_ = 0;
uint8_t temp_is_fresh_ = 0;
uint8_t debug_ = 0;
uint8_t monitor_temp_ = 0;
uint8_t pump_autoon_ = 0;
// at f_system_clk = 10Hz, system_clk_ will not overrun for at least 13 years. PCR won't run that long
volatile uint32_t system_clk_ = 0;

//with F_CPU = 16MHz and TIMER3 Prescaler set to /1024, TIMER3 increments with f = 16KHz. Thus if TIMER3 reaches 16, 1ms has passed.
#define T3_MS     *16
//set TICK_TIME to 1/10 of a second
#define SYSCLKTICK_DURATION_IN_MS 100
#define	TICK_TIME (SYSCLKTICK_DURATION_IN_MS T3_MS)

ISR(TIMER3_COMPA_vect)
{
  //increment system_clk every TIME_TICK (aka 100ms)
	system_clk_++;
  //set up "clock" comparator for next tick
  OCR3A = (OCR3A + TICK_TIME) & 0xFFFF;
  if (debug_)
    led_toggle();
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
    static uint32_t conversion_start_time = 0;
    uint8_t sensor_index = 0;

    if (num_temp_sensors_ == 0)
    {
        num_temp_sensors_ = ds1820_discover();
    }

    if (conversion_start_time == 0)
    {
      // -- trigger temperatur conversion in sensor
      for (sensor_index=0; sensor_index < num_temp_sensors_; sensor_index++)
      {
          ds1820_set_resolution(sensor_index, bit_resolution);
          ds1820_start_measuring(sensor_index);
      }
      conversion_start_time = system_clk_;
      // -- end trigger
    } else
    if ( (system_clk_ - conversion_start_time)*SYSCLKTICK_DURATION_IN_MS > ds1820_get_conversion_time_ms(bit_resolution))
    {
      // -- receive temperatur after conversion time has passed
      for (sensor_index=0; sensor_index < num_temp_sensors_; sensor_index++)
      {
          raw_temp_ = ds1820_read_temperature(sensor_index);
          if (raw_temp_ != DS1820_ERROR)
          {
              temp_is_fresh_ = 1;
              break; //we need only one successfully read value
          }
      }
      conversion_start_time = 0;
      // -- end receive
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
    printf("{\"cmd\":\"s\",\"cmd_ok\":false,\"error\": \"No DS1820 sensors on 1wire bus, thus no temperature\"}\r\n");
    return;
  }
  if (raw_temp_ == DS1820_ERROR)
  {
      printf("{\"cmd\":\"s\",\"cmd_ok\":false,\"error\":\"talking to DS18b20, no valid temperature!\"}\r\n");
  } else {
      printf("{\"cmd\":\"s\",\"t\":%lu, \"currtemp\":", system_clk_);
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
    PIN_LOW(PELTIER_INA_PORT, PELTIER_INA_PIN);
    PIN_HIGH(PELTIER_INB_PORT, PELTIER_INB_PIN);
    pwm_b5_set((uint8_t) value);
  } else {
    PIN_HIGH(PELTIER_INA_PORT, PELTIER_INA_PIN);
    PIN_LOW(PELTIER_INB_PORT, PELTIER_INB_PIN);
    pwm_b5_set((uint8_t) (-1 * value));
  }
  if (debug_)
    printf("Peltier value: %d, INA: %d, INB: %d, OCR1AH: %d, OCR1AL: %d\r\n", value, (PELTIER_INA_PORT & _BV(PELTIER_INA_PIN)) > 0, (PELTIER_INB_PORT & _BV(PELTIER_INB_PIN)) > 0, OCR1AH, OCR1AL);
}

void setTopHeaderValue(int16_t value)
{
    pwm_d7_set((uint8_t) (value & 0xFF));
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
  case 'M': monitor_temp_ = 1; break;
  case 'm': monitor_temp_ = 0; break;
  case '=': pid_setTargetValue(raw_temp_); break;
  case '#': pid_setTargetValue(PID_DISABLED); break;
  case 't':
  case 's': printStatus(); return;
  case 'L': led_toggle(); break;
  case 'l': cmdq_queueCmdWithNumArgs((void*) led_toggle, 0, cmd); return;
  case 'p':
  case 'i':
  case 'd':
    pid_printVars();
    return;
  case 'T': cmdq_queueCmdWithNumArgs((void*) pid_setTargetValue, 1, cmd); return;
  case 'P': cmdq_queueCmdWithNumArgs((void*) pid_setP, 1, cmd); return;
  case 'I': cmdq_queueCmdWithNumArgs((void*) pid_setI, 1, cmd); return;
  case 'D': cmdq_queueCmdWithNumArgs((void*) pid_setD, 1, cmd); return;
  case 'A':
    PIN_HIGH(PUMP_PORT, PUMP_PIN);
    pump_autoon_ = 0;
    break;
  case 'a':
    PIN_LOW(PUMP_PORT, PUMP_PIN);
    pump_autoon_ = 0;
    break;
  case 'B': cmdq_queueCmdWithNumArgs((void*) setTopHeaderValue, 1, cmd); return;
  case 'b': setTopHeaderValue(0); break;
  //~ case 'B': PIN_HIGH(TOPHEAT_PORT,TOPHEAT_PIN); break;
  //~ case 'b': PIN_LOW(TOPHEAT_PORT,TOPHEAT_PIN); break;
  case '@': pump_autoon_ = 1; break;
  case '.': tcurve_printCurve(cmd); return;
  case '-': //reset temp curve
    tcurve_reset();
    break;
  case '+': //add temp curve entry
    //~ tcurve_add(readNumber(), readNumber());
    cmdq_queueCmdWithNumArgs((void*) tcurve_add, 2, cmd);
    return;
  case '>': cmdq_queueCmdWithNumArgs((void*) tcurve_setRepeatStartPosToLatestEntry, 0, cmd); return;
  case '<': cmdq_queueCmdWithNumArgs((void*) tcurve_setRepeatEndPosToLatestEntry, 0, cmd); return;
  case 'Z': cmdq_queueCmdWithNumArgs((void*) tcurve_setRepeats, 1, cmd); return;
  default: printf("{\"cmd\":\"%c\",\"cmd_ok\":false,\"error\":\"unknown cmd\"}\r\n",cmd); return;
  }
  printf("{\"cmd\":\"%c\",\"cmd_ok\":true}\r\n",cmd);
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
  owi_init(ONEWIRE_PIN, &ONEWIRE_PINBASE);
  PINMODE_OUTPUT(PUMP_DDR, PUMP_PIN);
  PIN_LOW(PUMP_PORT, PUMP_PIN);
  PINMODE_OUTPUT(PELETIER_PWM_EN_DDR, PELETIER_PWM_EN_PIN);
  PINMODE_OUTPUT(PELTIER_INB_DDR, PELTIER_INB_PIN);
  PINMODE_OUTPUT(PELTIER_INA_DDR, PELTIER_INA_PIN);
  PINMODE_OUTPUT(TOPHEAT_DDR, TOPHEAT_PIN);
  PIN_LOW(TOPHEAT_PORT, TOPHEAT_PIN);

  pwm_init();
  pwm_b5_set(0);

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

    queryAndSaveTemperature(11); //at 11bit resolution

    if (temp_is_fresh_)
    {
      temp_is_fresh_ = 0; //once used, temp is used up ;->

      if (monitor_temp_)
        printStatus();

      if (tcurve_isSet())
      {
        uint16_t time_elapsed = (uint16_t) (system_clk_ - last_time);
        last_time = system_clk_;
        //PID_DISABLED == TCURVE_ERROR so this works out fine and we disable heating and PID in this case
        pid_setTargetValue(tcurve_getTempToSet(raw_temp_, time_elapsed));
      }

      // PID control
      // make sure this is called at exact periodic intervals (i.e. make sure there are no large variable delays in for loop)
      // e.g. enable periodic temp monitoring 'm' rather than querying temp at some intervall 's'
      if (pid_isEnabled())
      {
        if (pump_autoon_)
          PIN_HIGH(PUMP_PORT, PUMP_PIN);
        if (debug_)
          printf("pid_calc..");
        while (system_clk_ - last_time2 < 5); //wait until at least 500ms have passed since last time. Should be enough time for everything else to finish. (after 13 years, code will hang here)
        if (debug_)
          printf(" clk: %lu, elaps: %lu\r\n",system_clk_ , system_clk_ - last_time2);
        last_time2 = system_clk_;
        temp_is_fresh_ = 0;
        setPeltierCoolingDirectionPower(pid_calc(raw_temp_));
      }
      else
      {
        setPeltierCoolingDirectionPower(0);
        if (pump_autoon_)
          PIN_LOW(PUMP_PORT, PUMP_PIN);
      }
    }

    anyio_task();
  }
}
