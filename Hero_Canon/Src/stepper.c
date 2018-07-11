#include "stepper.h"
#include "tim.h"
#include "gpio.h"
#include "controlTask.h"

Stepper_Regulator_t stepper_left = STEPPER_LEFT_REGULATOR_DEFAULT;
Stepper_Regulator_t stepper_right = STEPPER_RIGHT_REGULATOR_DEFAULT;

void stepper_init(Stepper_Regulator_t * stp)
{
	stp->enable = 0;
	stp->pulses = 0;
	// cw pins and enable pins configuration GPIOC0&C1
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	switch(stp->id)
	{
		case STEPPER_LEFT:
		{
			if(stp->cw == CLOCKWISE)
				STEPPER_LEFT_CLOCKWISE();
			else
				STEPPER_LEFT_COUNTER_CLOCKWISE();
		}break;
		case STEPPER_RIGHT:
		{
			if(stp->cw == CLOCKWISE)
				STEPPER_RIGHT_CLOCKWISE();
			else
				STEPPER_RIGHT_COUNTER_CLOCKWISE();
		}break;
	}
	
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

	// timer configuration
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = stp->period;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = stp->period/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, stp->channel) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_TIM_MspPostInit(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
	stepper_stop(stp);
}


void stepper_apply(Stepper_Regulator_t * stp)
{
	// config direction
	switch(stp->id)
	{
		case STEPPER_LEFT:
		{
			if(stp->cw == CLOCKWISE)
				STEPPER_LEFT_CLOCKWISE();
			else
				STEPPER_LEFT_COUNTER_CLOCKWISE();
		}break;
		case STEPPER_RIGHT:
		{
			if(stp->cw == CLOCKWISE)
				STEPPER_RIGHT_CLOCKWISE();
			else
				STEPPER_RIGHT_COUNTER_CLOCKWISE();
		}break;
	}
	
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

	// timer configuration
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = stp->period;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = stp->period/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, stp->channel) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_MspPostInit(&htim2);
}

void stepper_enable(Stepper_Regulator_t *stp)
{
	switch(stp->id)
	{
		case STEPPER_LEFT:
		{
			STEPPER_LEFT_ENABLE();
		}break;
		case STEPPER_RIGHT:
		{
			STEPPER_RIGHT_ENABLE();
		}break;
	}
}

void stepper_disable(Stepper_Regulator_t *stp)
{
	switch(stp->id)
	{
		case STEPPER_LEFT:
		{
			STEPPER_LEFT_DISABLE();
		}break;
		case STEPPER_RIGHT:
		{
			STEPPER_RIGHT_DISABLE();
		}break;
	}
}

void stepper_setDir(Stepper_Regulator_t *stp,uint8_t cw)
{
	stp->cw = cw;
	switch(stp->id)
	{
		case STEPPER_LEFT:
		{
			if(stp->cw == CLOCKWISE)
				STEPPER_LEFT_CLOCKWISE();
			else
				STEPPER_LEFT_COUNTER_CLOCKWISE();
		}break;
		case STEPPER_RIGHT:
		{
			if(stp->cw == CLOCKWISE)
				STEPPER_RIGHT_CLOCKWISE();
			else
				STEPPER_RIGHT_COUNTER_CLOCKWISE();
		}break;
	}
}

void stepper_config(Stepper_Regulator_t *stp,uint8_t cw,uint16_t rpm)
{
	stp->cw = cw;
	stp->rpm = rpm;
	stp->period = (20000*60)/(FULL_REV*rpm); // 20000 = 84M/4200(DAFAULT TIM2 Prescaler)
	stepper_init(stp);
}

void stepper_start(Stepper_Regulator_t *stp)
{
	HAL_TIM_PWM_Start(&htim2,stp->channel);
	stp->enable = 1;
	stepper_enable(stp);
}

void stepper_stop(Stepper_Regulator_t *stp)
{
	HAL_TIM_PWM_Stop(&htim2,stp->channel);
	stp->enable = 0;
	stepper_disable(stp);
}

void stepper_hold(Stepper_Regulator_t *stp)
{
	if(stp->id == STEPPER_LEFT)
	{
		HAL_TIM_PWM_Start(&STEPPER_TIM,STEPPER_LEFT_CHANNEL);
	}
	else
	{
		HAL_TIM_PWM_Stop(&STEPPER_TIM,STEPPER_LEFT_CHANNEL);
	}
	
	if(stp->id == STEPPER_RIGHT)
	{
		HAL_TIM_PWM_Start(&STEPPER_TIM,STEPPER_RIGHT_CHANNEL);
	}
	else
	{
		HAL_TIM_PWM_Stop(&STEPPER_TIM,STEPPER_RIGHT_CHANNEL);
	}
}

void stepper_idle(Stepper_Regulator_t *stp)
{
}

void stepper_rotate(Stepper_Regulator_t *stp,uint8_t cw,float degree)
{
	if(stp->enable==0)
	{
		stepper_setDir(stp,cw);
		stp->pulses = degree/STEP_ANGLE;
		stp->mode = MODE_SERVO;
	}
}


void stepper_raise(Stepper_Regulator_t *stp,uint16_t mm)
{

}

void stepper_lower(Stepper_Regulator_t *stp, uint16_t mm)
{
}

void stepper_setHeight(Stepper_Regulator_t *stpr, uint16_t mm)
{
}
