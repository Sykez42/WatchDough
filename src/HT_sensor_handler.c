/*
 * HT_sensor_handler.c
 *
 *  Created on: 7 ott 2023
 *      Author: merca
 */

#include "HT_sensor_handler.h"

float temp_value = 0;  // Measured temperature value
struct temperature T_read;
float temperature;


struct temperature ReadTemperature(){


	temp_value = BSP_TSENSOR_ReadTemp();
	int tmpInt1 = temp_value;
	float tmpFrac = temp_value - tmpInt1;
	int tmpInt2 = trunc(tmpFrac * 100);
	BSP_TSENSOR_Init();
	T_read.temp_int = tmpInt1;
	T_read.temp_dec= tmpInt2;

	return T_read;


}

float ReadTemperature_float(){

	//code goes here

}
}

