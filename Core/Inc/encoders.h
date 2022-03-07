/*
 * encoders.h
 *
 *  Created on: Aug 11, 2021
 *      Author: Lukasz_Wiater
 */

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#include "main.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
uint8_t spi_error;

int16_t enc_langle;
int16_t enc_langle_old;
float enc_ldiff;
int16_t enc_ldiff_raw;

int16_t enc_rangle;
int16_t enc_rangle_old;
float enc_rdiff;
int16_t enc_rdiff_raw;


uint16_t ENCReading(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t data);
uint16_t ENCWriting(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t addr, uint16_t data);


#endif /* INC_ENCODERS_H_ */
