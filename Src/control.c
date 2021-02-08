/**
* @file         control.c
* @author       Weyne Chen
* @version      V01
* @date         2020.03.10
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#include "control.h"
#include "main.h"
#include "tim.h"
#include "pro_v2.h"
#include "pro.h"
#include <string.h>

typedef struct control
{
	int32_t tim_counter;
	int32_t set_pos;
	uint32_t period;
	int32_t curr_pos;
	uint8_t direction;
}ControlType;

typedef struct control_info
{
	int32_t pos;
	uint32_t limit_switch;
	uint32_t ir_light;
	uint32_t speed;
}CtlInfoType;

ControlType Control = {
	.period = TIMER_PERIOD,
};

uint32_t GetTimeUs(void)
{
    uint32_t systick_period = 1000;        
    uint32_t systick_load = SysTick->LOAD; 
    uint32_t systick_curr = systick_load - SysTick->VAL;
    uint32_t curr = HAL_GetTick() * 1000 + systick_curr * systick_period / systick_load;
    return curr;
}

void DelayUs(int us)
{
	int time = GetTimeUs();
	
	while(GetTimeUs() - time < us);
}

static void TimerFreqSet(uint32_t speed)
{
	if(speed >= TIMER_MIN_FREQ && speed <= TIMER_MAX_FREQ)
	{
		Control.period = SYS_CLOCK / speed;

		__HAL_TIM_SET_AUTORELOAD(&htim4, (Control.period - 1));
	}
}

void Motor_SetStep(int32_t pos, uint32_t speed)
{
	Control.direction = (pos >= 0) ? 0 : 1;
	Control.set_pos = pos * DISTANCE_COE;
	Control.tim_counter = 0;
	TimerFreqSet(speed);
}

void Motor_Output(void)
{
	
	if(Control.direction == 0)
	{
		TIM4->CCR1 = Control.period;
		TIM4->CCR2 = Control.period / 2;
		//TIM4->CCR1 = Control.period / 2;
		//TIM4->CCR2 = 0;
	}
	else
	{
		TIM4->CCR1 = 0;
    TIM4->CCR2 = Control.period / 2;
	}
}

void Motor_start(void)
{
	if(Control.set_pos == 0)
	{
		return;
	}

	if(HAL_GPIO_ReadPin(LIMIT_SW_Port, LIMIT_SW_Pin) == GPIO_PIN_RESET && Control.direction == 1)
	{
		return;
	}

	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void Motor_stop(void)
{
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
}

void Motor_StepControl(void)
{
	if(Control.direction == 0)
	{
		if (Control.curr_pos <= (200 * DISTANCE_COE)) {
			Control.curr_pos++;
		} else {
			Motor_stop();
			return;
		}
		
		Control.tim_counter++;
		if(Control.tim_counter >=  Control.set_pos)
		{
			Motor_stop();
		}
	}
	else
	{
		Control.curr_pos--;
		Control.tim_counter--;
		if(Control.tim_counter <= Control.set_pos)
		{
			Motor_stop();
		}
	}
}

void Motor_GetPos(void)
{
	static uint32_t curr_time = 0;
	
	if(HAL_GetTick() - curr_time > 20)
	{
		CtlInfoType info;

		info.pos = Control.curr_pos / DISTANCE_COE;
		info.limit_switch = HAL_GPIO_ReadPin(LIMIT_SW_Port, LIMIT_SW_Pin);
		info.ir_light = HAL_GPIO_ReadPin(LED_Port, LED_Pin);
		info.speed = SYS_CLOCK / (__HAL_TIM_GET_AUTORELOAD(&htim4) + 1);

		Comm_SendData(PACK_GET_POS, (uint8_t *)&info, sizeof(info));
	}
}

void Motor_ClearPos(void)
{
	Control.curr_pos = 0;
	Control.set_pos = 0;
	Control.tim_counter = 0;
}

void Motor_LimitSwitch(void)
{
	if(HAL_GPIO_ReadPin(LIMIT_SW_Port, LIMIT_SW_Pin) == GPIO_PIN_RESET && Control.direction == 1) 
	{
		DelayUs(1000);
		if(HAL_GPIO_ReadPin(LIMIT_SW_Port, LIMIT_SW_Pin) == GPIO_PIN_RESET){
			Motor_stop();
			Motor_ClearPos();
		}
	}
}

int32_t Motor_GetCurrPos(void)
{
	return Control.curr_pos / DISTANCE_COE;
}
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
