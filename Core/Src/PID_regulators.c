/*
 * PID_regulators.c
 *
 *  Created on: Aug 7, 2021
 *      Author: Lukasz_Wiater
 */


#include "PID_regulators.h"



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM3)
 {
	 PID_MotorRegulator();
 }
 if(htim->Instance == TIM4)
 {
	 PID_MainRegulator();
	 //lama++;
 }
}



void PID_MainRegulator(void)
{
	uint16_t pid_act_velocity=pid_basic_velocity;
	int16_t pid_u=0;
	int16_t pid_e=0;
	static int16_t pid_prev_e=0;
	uint16_t pid_e_straight=0;
	uint16_t pid_straight_threshold=10000;
	uint16_t pid_straight_velocity=3;
	static uint16_t pid_straight_counter=0;



	// uchyb ----------------
	pid_e = KTIR_ControlError();

	// sterowanie -----------
	if((pid_e>-pid_e_straight)&&(pid_e<pid_e_straight)) 		// jazda prosto - przyspieszenie
	{
		if(pid_straight_counter<=pid_straight_threshold)
		{
			pid_straight_counter++;
		}
		else
		{
			pid_act_velocity = pid_straight_velocity;
		}
	}
	else
	{
		pid_act_velocity = pid_basic_velocity;
	}

	if(pid_e==32000) 						// zgubienie trasy
	{
		if(pid_prev_e<0)
			pid_u=-pid_u_track_lost;
		if(pid_prev_e>0)
			pid_u=pid_u_track_lost;

		pid_target_l=pid_act_velocity+pid_u;
		pid_target_r=pid_act_velocity-pid_u;
		pid_straight_counter=0;
	}
	else									// normalna jazda
	{
		// regulator ------------
		pid_u=(pid_k_coefficient)*pid_e+(pid_k_coefficient*pid_Td_coefficient*(pid_e-pid_prev_e));

		pid_target_l=pid_act_velocity+pid_u;
		pid_target_r=pid_act_velocity-pid_u;

		pid_prev_e=pid_e;
		pid_straight_counter=0;
	}
}



void PID_MotorRegulator(void)
{

	//static int16_t prev_ui_l=0;
	static float prev_ui_r=0;
	static int16_t pid_lmotor_prev_e=0;
	static int16_t pid_rmotor_prev_e=0;
	//int16_t pid_lmotor_e=0;
	int16_t pid_rmotor_e=0;

	float up=0.0;
	float ui=0.0;
	float ud=0.0;


	// odczytanie wartosci kata absolutnego ------------------------------------------------------------

	enc_langle = (ENCReading(&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0xFFFF)) << 2;
	enc_rangle = (ENCReading(&hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0xFFFF)) << 2;

	enc_ldiff_raw = (enc_langle - enc_langle_old);
	enc_langle_old = enc_langle;
	enc_rdiff_raw = (enc_rangle - enc_rangle_old);
	enc_rangle_old = enc_rangle;

	enc_ldiff = PID_AlphaBetaFilterLeft(enc_ldiff_raw);
	enc_rdiff = PID_AlphaBetaFilterRight(enc_rdiff_raw);

	// regulator PID silnikow --------------------------------------------------------------------------

	// lewy silnik -----------

	pid_lmotor_e=pid_target_l-enc_ldiff;
	up=(pid_motor_k)*pid_lmotor_e;
	ui=prev_ui_l+((pid_motor_k*(pid_lmotor_prev_e+pid_lmotor_e))/(pid_motor_Ti*2));
	ud=(pid_motor_k*pid_motor_Td*(pid_lmotor_e-pid_lmotor_prev_e));
	prev_ui_l=ui;

	pid_lmotor_u=up+ui+ud;
	//pid_lmotor_u=up;
	pid_ltest=pid_lmotor_u;

	// prawy silnik -----------

	pid_rmotor_e=pid_target_r-enc_rdiff;
	up=(pid_motor_k)*pid_rmotor_e;
	ui=prev_ui_r+((pid_motor_k*(pid_rmotor_prev_e+pid_rmotor_e))/(pid_motor_Ti*2));
	ud=(pid_motor_k*pid_motor_Td*(pid_rmotor_e-pid_rmotor_prev_e));
	prev_ui_r=ui;

	pid_rmotor_u=up+ui+ud;
	//pid_rmotor_u=up;

	// ograniczenie sterowania

	if(pid_lmotor_u<-500)
		pid_lmotor_u = -500;
	if(pid_lmotor_u>500)
		pid_lmotor_u = 500;

	if(pid_rmotor_u<-500)
		pid_rmotor_u = -500;
	if(pid_rmotor_u>500)
		pid_rmotor_u = 500;


	if(pid_lmotor_u>-35&&pid_lmotor_u<35)
		pid_lmotor_u = 0;

	if(pid_rmotor_u>-35&&pid_rmotor_u<35)
		pid_rmotor_u = 0;

	// ustawienie predkosci ---

	MOTORS_SetVelocity(pid_lmotor_u, pid_rmotor_u);
	pid_lmotor_prev_e=pid_lmotor_e;
	pid_rmotor_prev_e=pid_rmotor_e;
	//TIM3->CNT=30000;
	//TIM4->CNT=30000;
}

float PID_AlphaBetaFilterLeft(int16_t measurement)
{
	static float x0=0;
	static float x1=0;
	static float v0=0;
	static float v1=0;
	static float r=0;
	float dt=0.000125;

	x1 = x0 + (v0 * dt);
	v1 = v0;

	r = measurement - x1;

	x1 = x1 + (pid_alpha * r);
	v1 = v1 + ((pid_beta*r) / dt);

	x0 = x1;
	v0 = v1;

	return x0;
}

float PID_AlphaBetaFilterRight(int16_t measurement)
{
	static float x0=0;
	static float x1=0;
	static float v0=0;
	static float v1=0;
	static float r=0;
	float dt=0.000125;

	x1 = x0 + (v0 * dt);
	v1 = v0;

	r = measurement - x1;

	x1 = x1 + (pid_alpha * r);
	v1 = v1 + ((pid_beta*r) / dt);

	x0 = x1;
	v0 = v1;

	return x0;
}
