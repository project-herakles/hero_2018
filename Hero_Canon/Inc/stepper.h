#ifndef STEPPER_H_
#define STEPPER_H_
#include "stm32f4xx.h"
#include "tim.h"

#define STEPPER_LEFT 1
#define STEPPER_RIGHT 2
#define STEPPER_LEFT_CHANNEL TIM_CHANNEL_3
#define STEPPER_RIGHT_CHANNEL TIM_CHANNEL_4
#define STEPPER_TIM htim2
#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0
// The number of pulses it takes to revolve one cycle
#define FULL_REV 200

#define STEPPER_LEFT_CLOCKWISE() 					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET)
#define STEPPER_LEFT_COUNTER_CLOCKWISE()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET)
#define STEPPER_RIGHT_CLOCKWISE()					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET)
#define STEPPER_RIGHT_COUNTER_CLOCKWISE() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET)

typedef struct
{
	uint8_t id; // [PRIVATE]
	uint8_t channel; // [PRIVATE]
	uint8_t enable;
	uint8_t cw;
	uint16_t rpm;
	uint32_t period; // [PRIVATE]
}Stepper_Regulator_t;

#define STEPPER_LEFT_REGULATOR_DEFAULT \
{ \
	STEPPER_LEFT, \
	STEPPER_LEFT_CHANNEL, \
	0, \
	0, \
	60, \
	100 \
} \

#define STEPPER_RIGHT_REGULATOR_DEFAULT \
{ \
	STEPPER_RIGHT, \
	STEPPER_RIGHT_CHANNEL, \
	0, \
	0, \
	60, \
	100 \
} \

void stepper_init(Stepper_Regulator_t * stp);
void stepper_config(Stepper_Regulator_t *stp,uint8_t cw,uint16_t rpm);
void stepper_start(Stepper_Regulator_t *stp);
void stepper_stop(Stepper_Regulator_t *stp);
void stepper_hold(Stepper_Regulator_t *stp);
void stepper_idle(Stepper_Regulator_t *stp);
void stepper_raise(Stepper_Regulator_t *stp,uint16_t mm);
void stepper_lower(Stepper_Regulator_t *stp, uint16_t mm);
void stepper_setHeight(Stepper_Regulator_t *stpr, uint16_t mm);

#endif
