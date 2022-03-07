/*
 * ktir.c
 *
 *  Created on: Aug 7, 2021
 *      Author: Lukasz_Wiater
 */


#include "ktir.h"


int16_t KTIR_ControlError(void)
{
	int16_t control_error = 0;
	uint16_t track_detected = 0;
	int16_t track_coefficients[KTIR_NUM]={-7,-6,-5,-60,-30,-10,10,30,60,5,6,7};
	uint8_t track_detection_array[KTIR_NUM]={0,0,0,0,0,0,0,0,0,0,0,0};

	for(int i=3;i<6+3;i++)
	{
		if(ktir_array[i]<KTIR_DETECTION_THRESHOLD)
			track_detection_array[i]=0;
		else
		{
			track_detection_array[i]=1;
			track_detected=1;
		}
	}

	if(track_detected==0)
	{
		return 32000;
	}
	else
	{
		for(int i=0;i<KTIR_NUM;i++)
		{
			control_error=control_error+(track_detection_array[i]*track_coefficients[i]);
			//control_error=control_error+2;
		}
	}

	return control_error;
}
