
#include "pwm.h"


#define PWM_VAL OCR1AL
//OC1A

void pwm_init(void)
{
  DDRB |= (1<<PB5);
  TCCR1A = 0;
  TCNT1 = 0;
  OCR1A = 0;
  TCCR1A = (1<<WGM10);  //Fast PWM, 8-bit
  TCCR1B = (1<<WGM12); //Fast PWM, 8-bit
  //TCCR1A |= (1<<COM1A1) // Clear OCnA/OCnB/OCnC on compare match when up-counting. Set OCnA/OCnB/OCnC on compare match when downcounting.
}

inline void pwm_on(void)
{
  TCCR1A |= (1<<COM1A1); // Clear OCnA/OCnB/OCnC on compare match when up-counting. Set OCnA/OCnB/OCnC on compare match when downcounting.
  TCCR1B = (TCCR1B & 0xF8) | (1<<CS10); // enable timer clock, no prescaling
}

inline void pwm_off(void)
{
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0)); //normal port operation
  TCCR1B = (TCCR1B & 0xF8);  //no clock source, timer stopped
  TCNT1 = 0;
  PORTB &= ~ (1 << PB5);  //set pin to LOW (otherwise it will remain in state since last pwm toggle)
}

void pwm_set(uint8_t val)
{
  if (val > 0)
    pwm_on();
  else
    pwm_off();
  PWM_VAL = val;
}

inline void pwm_inc(void)
{
  if(PWM_VAL < 255)
    PWM_VAL++;
}

inline void pwm_dec(void)
{
  if(PWM_VAL > 0)
    PWM_VAL--;
}
