/*
 * PID_regulators.h
 *
 *  Created on: Aug 7, 2021
 *      Author: Lukasz_Wiater
 */

#ifndef INC_PID_REGULATORS_H_
#define INC_PID_REGULATORS_H_

#include "main.h"
#include "ktir.h"
#include "encoders.h"
#include "motors.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

// główny regulator
float pid_k_coefficient;
float pid_Td_coefficient;
uint16_t pid_basic_velocity;
uint16_t pid_u_track_lost;


// motor regulator
float pid_motor_k;
uint16_t pid_motor_Ti;
float pid_motor_Td;

int16_t pid_target_l;
int16_t pid_target_r;

int16_t pid_lmotor_u;
int16_t pid_rmotor_u;

// filtr alfa-beta
float pid_alpha;
float pid_beta;

int16_t pid_ltest;
float temp01;
float prev_ui_l;
int16_t pid_lmotor_e;

void PID_MainRegulator(void);
void PID_MotorRegulator(void);
float PID_AlphaBetaFilterLeft(int16_t measurement);
float PID_AlphaBetaFilterRight(int16_t measurement);

#endif /* INC_PID_REGULATORS_H_ */
