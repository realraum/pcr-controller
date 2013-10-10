#ifndef PWM_H_INCLUDED_
#define PWM_H_INCLUDED_

#include <avr/io.h>

void pwm_init(void);
void pwm_set(uint8_t val);


#endif