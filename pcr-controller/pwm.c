
#include "pwm.h"


#define PWM_VAL OCR1AL
//OC1A

void pwm_init(void)
{
  DDRB |= (1<<PB5);
  TCCR1A = 0;
  TCNT1 = 0;
  OCR1A = 0;
  TCCR1A = (1<<COM1A1) | (1<<WGM10);
  TCCR1B = (1<<WGM12);
}

inline void pwm_on(void)
{
  TCCR1B = (TCCR1A & 0xF8) | (1<<CS10);
}

inline void pwm_off(void)
{
  TCCR1B = (TCCR1A & 0xF8);
  TCNT1 = 0;
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
