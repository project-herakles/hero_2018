#ifndef LIFTER_H_
#define LIFTER_H_
#include "stm32f4xx.h"
#include "stepper.h"

#define LIFT_STAGE1 1
#define LIFT_STAGE2 2
#define LIFT_STAGE3 3

// unit: millionmeters
#define HEIGHT0 0
#define HEIGHT1 84
#define HEIGHT2 100

// unit: millionseconds
#define LATCH_DELAY 1000

void Lift_init(void);
void Lift_setHeight(uint16_t mm);
void Lift_hold(void);
void Lift_idle(void);
uint16_t getCurrentHeight(void);

#endif
