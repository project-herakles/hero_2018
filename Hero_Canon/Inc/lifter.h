#ifndef LIFTER_H_
#define LIFTER_H_
#include "stm32f4xx.h"
#include "stepper.h"

#define LIFT_STAGE1 1
#define LIFT_STAGE2 2
#define LIFT_STAGE3 3

// unit: millionmeters
#define HEIGHT1_2 84
#define HEIGHT2_3 100

uint16_t currentHeight;

void lift_setHeight(uint16_t mm);
void lift_hold(void);
void lift_idle(void);

#endif
