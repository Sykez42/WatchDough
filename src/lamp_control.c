/*
 * lamp_control.c
 *
 *  Created on: 7 ott 2023
 *      Author: merca
 */

#ifndef SRC_LAMP_CONTROL_C_
#define SRC_LAMP_CONTROL_C_


#include "lamp_control.h"


float local_target_temp;

void Lamp_OFF(void){

	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
	//assuming NO relay

};

void Lamp_ON(void){

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
	//assuming NO relay
};

void Auto_lamp_control(float target_temp, float sensor_temp){

	local_target_temp = target_temp + LOCAL_TEMP_DELTA;

	if (sensor_temp < (local_target_temp - LAMP_HYSTERESIS))
	{

		Lamp_ON();

	}
	else if (sensor_temp > (local_target_temp + LAMP_HYSTERESIS))
	{
		Lamp_OFF();

	}



};

#endif /* SRC_LAMP_CONTROL_C_ */
