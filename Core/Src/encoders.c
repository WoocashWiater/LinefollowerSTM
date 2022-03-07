/*
 * encoders.c
 *
 *  Created on: Aug 11, 2021
 *      Author: Lukasz_Wiater
 */


#include "encoders.h"

uint16_t ENCReading(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t addr)
{
	uint16_t rx_data = 0;
	//uint16_t tx_data = 0x7FFe;

	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive(spi, &addr, &rx_data, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

	return (rx_data&0x3FFF);
}

uint16_t ENCWriting(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t addr, uint16_t data)
{
	uint16_t rx_data = 0;
	//uint16_t tx_data = 0x7FFe;

	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(spi, &addr, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive(spi, &data, &rx_data, 1, HAL_MAX_DELAY)!=HAL_OK)
		spi_error=1;
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

	return (rx_data&0x3FFF);
}
