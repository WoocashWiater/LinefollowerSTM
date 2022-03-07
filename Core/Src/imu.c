/*
 * imu.c
 *
 *  Created on: Aug 28, 2021
 *      Author: Lukasz_Wiater
 */

#include "imu.h"


void IMUConfigureModule(void)
{
	uint8_t tx[]={0x70|0x80, 0};
	uint8_t rx[]={0};



	// ustawienie protokołu komunikacyjnego SPI (rejestr I2C_IF) ----------------------

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	if(HAL_SPI_Transmit(&hspi3, tx, 1, HAL_MAX_DELAY)!=HAL_OK)
	  	spi_error=1;
	if(HAL_SPI_Receive(&hspi3, rx, 1, HAL_MAX_DELAY)!=HAL_OK)
	  	spi_error=1;

	tx[0]=0x70;
	tx[1]=rx[0]|0b01000000;

	if(HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	// ustawienie rejestru CONFIG ------------------------------------------------------

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	tx[0]=0x1A;
	tx[1]=0x00;

	if(HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);


	// ustawienie rejestru PWR_MGMT_1 ------------------------------------------------------

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	tx[0]=0x6B;
	tx[1]=0b00011001; // do not sleep, gyro standby, temperature sensor disabled, (internal oscillator 20MHz-000, choose best-001)

	if(HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);


	// ustawienie rejestru PWR_MGMT_2 ------------------------------------------------------

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	tx[0]=0x6C|0x80;

	if(HAL_SPI_Transmit(&hspi3, tx, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	if(HAL_SPI_Receive(&hspi3, rx, 1, HAL_MAX_DELAY)!=HAL_OK)
	  	spi_error=1;

	tx[0]=0x6C;
	tx[1]=rx[0]|0b00000111; // accelerometers on, gyros off

	if(HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);


	// ustawienie rejestru ACCEL_CONFIG2 -----------------------------------------------

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	tx[0]=0x1D|0x80;

	if(HAL_SPI_Transmit(&hspi3, tx, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	if(HAL_SPI_Receive(&hspi3, rx, 1, HAL_MAX_DELAY)!=HAL_OK)
	  	spi_error=1;

	tx[0]=0x1D;
	tx[1]=rx[0]|0b00000110; // rate 1 kHz, lowest noise

	if(HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);


	// ustawienie rejestru SMPLRT_DIV ------------------------------------------------------

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	tx[0]=0x19;
	tx[1]=0b01100011; // 1000:10[internal_sample_rate/(1+smplrt_div)] (100Hz)

	if(HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);


	// ustawienie rejestru  ACCEL_INTEL_CTRL -------------------------------------------------

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	tx[0]=0x69|0x80;

	if(HAL_SPI_Transmit(&hspi3, tx, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	if(HAL_SPI_Receive(&hspi3, rx, 1, HAL_MAX_DELAY)!=HAL_OK)
	  	spi_error=1;

	tx[0]=0x69;
	tx[1]=rx[0]|0b00000010;

	if(HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);


	// ustawienie rejestru INT_PIN_CFG -------------------------------------------------

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	tx[0]=0x37|0x80;

	if(HAL_SPI_Transmit(&hspi3, tx, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	if(HAL_SPI_Receive(&hspi3, rx, 1, HAL_MAX_DELAY)!=HAL_OK)
	  	spi_error=1;

	tx[0]=0x37;
	tx[1]=rx[0]|0b00010000; // interrupt pin held until read, any read operation clears interrupt status

	if(HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

}

uint8_t IMUReadByte(uint8_t addr)
{
	uint8_t rx=0;
	addr=addr|0x80;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	if(HAL_SPI_Transmit(&hspi3, &addr, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	if(HAL_SPI_Receive(&hspi3, &rx, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	return rx;

}

int16_t IMURead2BVar(uint8_t addr)
{
	int16_t data;
	uint8_t rx[]={0,0};
	addr=addr|0x80;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	if(HAL_SPI_Transmit(&hspi3, &addr, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	if(HAL_SPI_Receive(&hspi3, rx, 2, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	data=(rx[0]<<8)+rx[1];

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	return data;
}

float IMUReadAcc(uint8_t axis)
{
	switch (axis)		// 1 - oś X, 2- oś Y, 3 - oś Z
	{
	case 1:
		return ((float)IMURead2BVar(0x3B)*ACC_RESOLUTION)/(float)INT16_MAX;
		break;
	case 2:
		return ((float)IMURead2BVar(0x3D)*ACC_RESOLUTION)/(float)INT16_MAX;
		break;
	case 3:
		return ((float)IMURead2BVar(0x3F)*ACC_RESOLUTION)/(float)INT16_MAX;
		break;
	default:
		return 0.0;
		break;
	}
}

void IMUReadAllAccData(void)
{
	uint8_t rx[6];
	int16_t data[3];
	uint8_t addr = 0x3B|0x80;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

	if(HAL_SPI_Transmit(&hspi3, &addr, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	if(HAL_SPI_Receive(&hspi3, rx, 6, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	data[0]=(rx[0]<<8)+rx[1];	// oś X
	data[1]=(rx[2]<<8)+rx[3];	// oś Y
	data[2]=(rx[4]<<8)+rx[5];	// oś Z

	imu_acc_data[0]=((float)data[0]*ACC_RESOLUTION)/(float)INT16_MAX;
	imu_acc_data[1]=((float)data[1]*ACC_RESOLUTION)/(float)INT16_MAX;
	imu_acc_data[2]=((float)data[2]*ACC_RESOLUTION)/(float)INT16_MAX;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == IMU_INT_Pin)
	{
		test3_imu=IMUReadByte(0x3A);
		test_counter++;
	}
}
