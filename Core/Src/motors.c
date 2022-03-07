/*
 * motors.c
 *
 *  Created on: Aug 3, 2021
 *      Author: Lukasz_Wiater
 */

#include "motors.h"



void MOTORS_SetVelocity(int16_t left_motor, int16_t right_motor)
{
	if(left_motor>=0)
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		if(left_motor>1000)
			left_motor=1000;
	}
	else
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		if(left_motor<-1000)
			left_motor=-1000;
	}

	if(right_motor>=0)
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		if(right_motor>1000)
			right_motor=1000;
	}
	else
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
		if(right_motor<-1000)
			right_motor=-1000;
	}

	TIM1->CCR1 = left_motor;
	TIM2->CCR1 = right_motor;

}
