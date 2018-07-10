#include "testTask.h"
#include "gpio.h"
#include "tim.h"
#include "can.h"
#include "main.h"
#include "stepper.h"

extern uint8_t canTxMsg[8];
extern UART_HandleTypeDef huart1;
extern uint8_t RemoteBuffer[18];
extern Stepper_Regulator_t stepper_left;
extern Stepper_Regulator_t stepper_right;
extern uint32_t time_tick_ms;

static uint8_t clockwise = 1;

void Test_Task_Init(void)
{
	stepper_init(&stepper_left);
	stepper_init(&stepper_right);
	RED_LED_ON();
}

void Test_Task(void)
{
	GREEN_LED_TOGGLE();
	RED_LED_OFF();
	static uint8_t waveLength = 1;
	if(waveLength == 4)
		waveLength = 0;
	waveLength++;
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	configPWMchannel(&htim2,TIM_CHANNEL_1,waveLength);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	if(time_tick_ms%3000==0)
	{
		if(clockwise == CLOCKWISE)
		{
			stepper_config(&stepper_left,COUNTER_CLOCKWISE,60);
			stepper_start(&stepper_left);
			clockwise = COUNTER_CLOCKWISE;
		}
		else
		{
			stepper_config(&stepper_left,CLOCKWISE,30);
			stepper_start(&stepper_left);
			clockwise = CLOCKWISE;
		}
		
	}
	
	//CAN_SendMsg(&hcan1,canTxMsg);
}
