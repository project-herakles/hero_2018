#ifndef __RAMP_H_
#define __RAMP_H_
#include "controlTask.h"
#include "pid.h"
#include <math.h>
#include <stdint.h>

void getRampedRef(PID_Regulator_t* pid,float step_ref,float slope);
uint16_t FrictionRamp(uint16_t start,uint16_t end,uint32_t duration);

#endif
