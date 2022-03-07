/*
 * motors.h
 *
 *  Created on: Aug 3, 2021
 *      Author: Lukasz_Wiater
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"

// zmienne globalne
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;


// deklaracje funkcji
void MOTORS_SetVelocity(int16_t left_motor, int16_t right_motor);

#endif /* INC_MOTORS_H_ */
