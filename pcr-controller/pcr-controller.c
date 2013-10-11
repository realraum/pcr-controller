/*
 *  OpenPCR Teensy Controller Code
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
#include <stdlib.h>

#include "util.h"
#include "led.h"
#include "anyio.h"

#include "onewire.h"
#include "ds1820.h"

#include "pwm.h"
#include "pid_control.h"

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

void printTemperature(void)
{
  if (num_temp_sensors_ == 0)
  {
    printf("ERROR: No DS1820 sensors on 1wire bus, thus no temperature\r\n");
    return;
  }
  if (raw_temp_ == DS1820_ERROR)
  {
      printf("ERROR talking to DS18b20, no valid temperature!\r\n");
  } else {
      printf("Temp: ");
      printRawTemp(raw_temp_);
      printf("\r\n");
  }
}

void readIntoBuffer(char *buffer, uint8_t buflen)
{
  while (anyio_bytes_received() == 0);
  int ReceivedByte=0;
  do {
    ReceivedByte = fgetc(stdin);
    if (ReceivedByte != EOF)
    {
      *buffer = (char) ReceivedByte;
      buffer++;
      buflen --;
    }
  } while (ReceivedByte != '\n' && ReceivedByte != '\r' && buflen > 1);
  *buffer = 0;
}

int16_t readNumber(void)
{
  char buffer[20];
  readIntoBuffer(buffer, 20);
  return atoi(buffer);
}

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
  case '=': pid_setTargetValue(raw_temp_); break;
  case '#': pid_setTargetValue(PID_DISABLED); break;
  case 's': printTemperature(); return;
  case 'L': led_toggle(); break;
  case 't':
    printf("TargetTemp: ");
    printRawTemp(pid_getTargetValue());
    printf("\r\n");
    return;
  case 'p': 
  case 'i':
  case 'd':
    pid_printVars();
    return;
  case 'T': pid_setTargetValue(readNumber()); break;
  case 'P': pid_setP(readNumber()); break;
  case 'I': pid_setI(readNumber()); break;
  case 'D': pid_setD(readNumber()); break;
  case 'A': PIN_HIGH(PORTB, PUMP_PIN); break;
  case 'a': PIN_LOW(PORTB, PUMP_PIN); break;  
  case 'B': PIN_HIGH(PORTD, TOPHEAT_PIN); break;
  case 'b': PIN_LOW(PORTD, TOPHEAT_PIN); break;  
  default: printf("ERROR\r\n"); return;
  }
  printf("OK\r\n");
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
  
  for(;;) 
  {
    int16_t BytesReceived = anyio_bytes_received();
    while(BytesReceived > 0)
    {
      int ReceivedByte = fgetc(stdin);
      if (ReceivedByte != EOF)
      {
        handle_cmd(ReceivedByte);
      }
      BytesReceived--;
    }
    
    queryAndSaveTemperature(11);
    
    // PID control
    // FIXME: if we do USB Input / Output (input especially) we delay PID controll too mauch
    //              that's bad, since the routing requires that it be called at exact intervalls
    //              maybe we should use a interrupt routine

    if (pid_isEnabled())
    {
      setPeltierCoolingDirectionPower(pid_calc(raw_temp_));
    }
    
    anyio_task();
  }
}
