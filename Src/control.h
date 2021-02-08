/**
* @file         control.h
* @author       Weyne Chen
* @version      V01
* @date         2020.03.10
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#ifndef __CONTROL_H
#define __CONTROL_H
#include <stdint.h>

#define DISTANCE_COE 835 //步进电机实际距离换算系数

void Motor_SetStep(int32_t pos, uint32_t speed);
void Motor_Output(void);
void Motor_StepControl(void);
void Motor_stop(void);
void Motor_start(void);
void Motor_GetPos(void);
void Motor_ClearPos(void);
void Motor_LimitSwitch(void);
int32_t Motor_GetCurrPos(void);

#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
