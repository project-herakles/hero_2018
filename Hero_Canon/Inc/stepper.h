#ifndef STEPPER_H_
#define STEPPER_H_
#include "stm32f4xx.h"

#define STEPPER_LEFT 1
#define STEPPER_RIGHT 2

void stepper_hold(int stepper);
void stepper_idle(int stepper);
void stepper_raise(int stepper,uint16_t mm);
void stepper_lower(int stepper, uint16_t mm);
void stepper_setHeight(int stepper, uint16_t mm);

#endif
