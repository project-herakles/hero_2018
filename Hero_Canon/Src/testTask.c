#include "testTask.h"
#include "gpio.h"
#include "tim.h"
#include "can.h"
#include "main.h"

extern uint8_t canTxMsg[8];
extern UART_HandleTypeDef huart1;
extern uint8_t RemoteBuffer[18];

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
	//CAN_SendMsg(&hcan1,canTxMsg);
}
