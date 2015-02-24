
#include "pwm.h"

//OutputCompareRegister for Timer1 and pin OC1A/B5
#define PWMB5_VAL OCR1AL
//for FastPWM in Timer4 OCR4C is TOP for all OC4x pins, all other PWM Modes use OCR4A,OCR4B or OCR4D
#define PWMD7_VAL OCR4D

void pwm_init(void)
{
  //for OC1A on pin B5
  DDRB |= (1<<PB5);
  TCCR1A = 0;
  TCNT1 = 0;
  OCR1A = 0;
  TCCR1A = (1<<WGM10);  //Fast PWM, 8-bit
  TCCR1B = (1<<WGM12); //Fast PWM, 8-bit
  //TCCR1A |= (1<<COM1A1) // Clear OCnA/OCnB/OCnC on compare match when up-counting. Set OCnA/OCnB/OCnC on compare match when downcounting. (for FastPWM)

  //for OC4D on pin D7
  DDRD |= (1<<PD7);
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = _BV(PWM4D); //Fast PWM: PWM4x == 0x1 && WGM41..40 ==0x00
  TCCR4D = 0; //Fast PWM: PWM4x == 0x1 && WGM41..40 ==0x00
  TCNT4 = 0;
  OCR4C = 0xFF; //TOP value
  OCR4D = 0;
}

inline void pwm_b5_on(void)
{
  TCCR1A |= (1<<COM1A1); // Clear OCnA/OCnB/OCnC on compare match when up-counting. Set OCnA/OCnB/OCnC on compare match when downcounting.
  TCCR1B = (TCCR1B & 0xF8) | (1<<CS10); // enable timer clock, no prescaling
}

inline void pwm_d7_on(void)
{
  TCCR4B = (TCCR4B & 0xF0) | (1<<CS40); // enable timer clock, no prescaling
  TCCR4C |=  (1<<COM4D1); // Clear OC4D on compare match when up-counting. Set when when TCNT4 == 0x000 (for FastPWM)
}

inline void pwm_b5_off(void)
{
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0)); //normal port operation
  TCCR1B = (TCCR1B & 0xF8);  //no clock source, timer stopped
  TCNT1 = 0;
  PORTB &= ~(1 << PB5);  //set pin to LOW (otherwise it will remain in state since last pwm toggle)
}

inline void pwm_d7_off(void)
{
  TCCR4B = (TCCR4B & 0xF0);  //no clock source, timer stopped
  TCCR4C &=  ~(_BV(COM4D1) | _BV(COM4D0)); //normal port operation
  TCNT4 = 0;
  PORTD &= ~(1 << PD7);  //set pin to LOW (otherwise it will remain in state since last pwm toggle)
}

void pwm_b5_set(uint8_t val)
{
  if (val > 0)
    pwm_b5_on();
  else
    pwm_b5_off();
  PWMB5_VAL = val;
}

void pwm_d7_set(uint8_t val)
{
  if (val > 0)
    pwm_d7_on();
  else
    pwm_d7_off();
  PWMD7_VAL = val;
}

inline void pwm_b5_inc(void)
{
  if(PWMB5_VAL < 255)
    PWMB5_VAL++;
}

inline void pwm_b5_dec(void)
{
  if(PWMB5_VAL > 0)
    PWMB5_VAL--;
}
