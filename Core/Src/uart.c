/*
 * uart.c
 *
 *  Created on: Aug 4, 2021
 *      Author: Lukasz_Wiater
 */

#include "uart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch(uart_received[0])
	{
		case 't':
			HAL_TIM_Base_Start_IT(&htim3);
			HAL_TIM_Base_Start_IT(&htim4);
			//mode_ride=1;
			//lama=50;
			break;

		case 'p':
			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_TIM_Base_Stop_IT(&htim4);
			MOTORS_SetVelocity(0, 0);
			MOTORS_SetVelocity(0, 0);
			//mode_ride=0;
			break;

		case 'b':
			//motor_basic_speed=UART_FrameToNumber(uart_received);
			break;

		case 'k':
			if(uart_received[1]=='g')
			{
				if(uart_received[2]=='.')
					pid_k_coefficient=UART_FrameToFloat3(uart_received);
				else
					pid_k_coefficient=UART_FrameToNumber(uart_received);
			}
			if(uart_received[1]=='m')
			{
				if(uart_received[2]=='.')
					pid_motor_k=UART_FrameToFloat(uart_received);
				else
					pid_motor_k=UART_FrameToNumber(uart_received);
			}
			break;

		case 'd':
			if(uart_received[1]=='g')
				pid_Td_coefficient=UART_FrameToFloat3(uart_received);
			if(uart_received[1]=='m')
				pid_motor_Td=UART_FrameToNumber(uart_received);
			break;

		case 'i':
			if(uart_received[1]=='m')
				pid_motor_Ti=UART_FrameToNumber(uart_received);
			break;

		case 's':
			//track_straight_threshold=UART_FrameToNumber(uart_received);
			break;

		case 'l':
			//track_lost=UART_FrameToNumber(uart_received);
			break;

		case 'v':
			if(uart_received[1]=='g')
				MOTORS_SetVelocity(UART_FrameToNumber(uart_received), UART_FrameToNumber(uart_received));
			if(uart_received[1]=='l')
				pid_target_l=UART_FrameToNumber(uart_received);
			if(uart_received[1]=='r')
				pid_target_r=UART_FrameToNumber(uart_received);
			break;

		case 'x':
			if(uart_received[1]=='g')
				MOTORS_SetVelocity(UART_FrameToNumber(uart_received), UART_FrameToNumber(uart_received));
			if(uart_received[1]=='l')
				MOTORS_SetVelocity(UART_FrameToNumber(uart_received), 0);
			if(uart_received[1]=='r')
				MOTORS_SetVelocity(0, UART_FrameToNumber(uart_received));
			break;

		case 'u':
			if(uart_received[1]=='1')
			{
				uart_size=sprintf(uart_data, "pid_k_coefficient: %4.2f \n\rpid_Td_coefficient: %4.2f \n\r", pid_k_coefficient, pid_Td_coefficient);
				HAL_UART_Transmit_IT(&huart1, uart_data, uart_size);
			}
			if(uart_received[1]=='2')
			{
				uart_size=sprintf(uart_data, "pid_motor_k: %4.8f \n\rpid_motor_Ti: %d \n\rpid_motor_Td: %4.8f \n\r", pid_motor_k, pid_motor_Ti, pid_motor_Td);
				HAL_UART_Transmit_IT(&huart1, uart_data, uart_size);
			}
			if(uart_received[1]=='3')
			{
				uart_size=sprintf(uart_data, "pid_alpha: %4.8f \n\rpid_beta: %4.8f \n\r", pid_alpha, pid_beta);
				HAL_UART_Transmit_IT(&huart1, uart_data, uart_size);
			}

			break;

		case 'f':
			if(uart_received[1]=='a')
				pid_alpha=UART_FrameToFloat(uart_received);
			if(uart_received[1]=='b')
				pid_beta=UART_FrameToFloat2(uart_received);
			break;

	}

	//uart_size=sprintf(uart_data, "OK\n\r");
	//HAL_UART_Transmit_IT(&huart3, uart_data, uart_size);
	HAL_UART_Receive_IT(&huart1, uart_received, 6);
	//lama=100;

}

uint16_t UART_FrameToNumber(uint8_t frame[])
{
	return (frame[5]-'0')+(10*(frame[4]-'0'))+(100*(frame[3]-'0'))+(1000*(frame[2]-'0'));
}

float UART_FrameToFloat(uint8_t frame[])
{
	return (((frame[5]-'0')+(10*(frame[4]-'0'))+(100*(frame[3]-'0')))/1000.0f);
}

float UART_FrameToFloat2(uint8_t frame[])
{
	return (((frame[5]-'0')+(10*(frame[4]-'0'))+(100*(frame[3]-'0')))/1000000000.0f);
}

float UART_FrameToFloat3(uint8_t frame[])
{
	return (((frame[5]-'0')+(10*(frame[4]-'0'))+(100*(frame[3]-'0')))/10.0f);
}

