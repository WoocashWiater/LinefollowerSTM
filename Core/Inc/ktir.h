/*
 * ktir.h
 *
 *  Created on: Aug 7, 2021
 *      Author: Lukasz_Wiater
 */

#ifndef INC_KTIR_H_
#define INC_KTIR_H_

#include "main.h"

#define KTIR_NUM 12
#define KTIR_DETECTION_THRESHOLD 3500

//uint32_t ktir_array[KTIR_NUM];
uint16_t ktir_array[KTIR_NUM];
//uint8_t track_detection_array[KTIR_NUM];


int16_t KTIR_ControlError(void);

#endif /* INC_KTIR_H_ */
