/*
 * imu.h
 *
 *  Created on: Aug 28, 2021
 *      Author: Lukasz_Wiater
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include "limits.h"

#define G_CONST 9.80665
#define ACC_RESOLUTION 2.0

SPI_HandleTypeDef hspi3;
uint8_t spi_error;
float imu_acc_data[3];

int16_t test_imu;
float test2_imu;
uint8_t test3_imu;
int16_t test_counter;


void IMUConfigureModule(void);
uint8_t IMUReadByte(uint8_t addr);
int16_t IMURead2BVar(uint8_t addr);
float IMUReadAcc(uint8_t axis);
void IMUReadAllAccData(void);


#endif /* INC_IMU_H_ */
