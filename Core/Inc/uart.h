/*
 * uart.h
 *
 *  Created on: Aug 4, 2021
 *      Author: Lukasz_Wiater
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include "PID_regulators.h"

// zmienne globalne
UART_HandleTypeDef huart1;
HAL_StatusTypeDef ret;
uint16_t uart_size;
uint8_t uart_data[200];
uint8_t uart_received[6];

//deklaracje funkcji
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint16_t UART_FrameToNumber(uint8_t frame[]);
float UART_FrameToFloat(uint8_t frame[]);
float UART_FrameToFloat2(uint8_t frame[]);
float UART_FrameToFloat3(uint8_t frame[]);


#endif /* INC_UART_H_ */
