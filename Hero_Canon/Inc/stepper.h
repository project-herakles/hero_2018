//		This is a stepper driver dedicated to HKU_Robomaster use
//		The steppers are managed by the Stepper_Regulator_t struct. Struct members are as follow:
//		-id: user defined variable to identify a stepper motor
//		-channel: the PWM channel connected to the stepper motor
//		-enable: private variable for enabling or disabling
//		-cw : short hand for counterwise, specify the direction of rotation, this can be either CLOCKWISE or COUNTER_CLOCKWISE
//		-rpm£º the speed at which the stepper rotates.
//		-period: a private variable to setup the period (hence PWM frequency) of the timer
//		-pulses: a private variable to calculate the angular displacement
//    -mode: a private variable to let the TIMER_IRQ handler decides whether it should stop the motor when pulses go to 0

//		There are typically 2 approaches to controling the                               																									
//		1. using stepper_start() and stepper_stop() pair. You may configure the speed, rotational direction by using stepper_config()
//    2. using stepper_rotate(). You may also use stepper_config() to configure the rotational speed
//    Note: You may use both approaches. But do not use them concurrently since it may incur undefined behaviour
//    			Do not config the stepper if one of them is rotating
//    Caution: This is not a full driver and you do need to manually configure the TIMER to get it working. Some important
// 						 parameters (declared as Macros) may be obliged to change due to different application

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
#define MODE_MOTOR 0
#define MODE_SERVO 1
// The number of pulses it takes to revolve one cycle
#define FULL_REV 200
#define RADIUS 15.0f
#define RPM2RADIAN 0.1047
#define STEP_ANGLE 1.8f

#define STEPPER_LEFT_ENABLE()							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)
#define STEPPER_RIGHT_ENABLE()						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET)
#define STEPPER_LEFT_DISABLE()						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET)
#define STEPPER_RIGHT_DISABLE()						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET)
#define STEPPER_LEFT_CLOCKWISE() 					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET)
#define STEPPER_LEFT_COUNTER_CLOCKWISE()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET)
#define STEPPER_RIGHT_CLOCKWISE()					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET)
#define STEPPER_RIGHT_COUNTER_CLOCKWISE() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET)

typedef struct
{
	uint8_t id; // [PRIVATE]
	uint8_t channel; // [PRIVATE]
	uint8_t enable;
	uint8_t cw;
	uint16_t rpm;
	uint32_t period; // [PRIVATE]
	uint32_t pulses;
	uint8_t mode;
}Stepper_Regulator_t;

#define STEPPER_LEFT_REGULATOR_DEFAULT \
{ \
	STEPPER_LEFT, \
	STEPPER_LEFT_CHANNEL, \
	0, \
	0, \
	60, \
	100, \
	0, \
} \

#define STEPPER_RIGHT_REGULATOR_DEFAULT \
{ \
	STEPPER_RIGHT, \
	STEPPER_RIGHT_CHANNEL, \
	0, \
	0, \
	60, \
	100, \
	0, \
} \

void stepper_init(Stepper_Regulator_t * stp);
void stepper_config(Stepper_Regulator_t *stp,uint8_t cw,uint16_t rpm);
void stepper_start(Stepper_Regulator_t *stp);
void stepper_stop(Stepper_Regulator_t *stp);
void stepper_hold(Stepper_Regulator_t *stp);
void stepper_idle(Stepper_Regulator_t *stp);
void stepper_rotate(Stepper_Regulator_t *stp,uint8_t cw, float degree);
void stepper_enable(Stepper_Regulator_t *stp);
void stepper_disable(Stepper_Regulator_t *stp);

#endif
