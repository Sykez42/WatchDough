/*
 * HT_sensor_handler.h
 *
 *  Created on: 7 ott 2023
 *      Author: merca
 */


#ifndef INC_HT_SENSOR_HANDLER_H_
#define INC_HT_SENSOR_HANDLER_H_

#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "commons.h"
#include <math.h>

struct temperature ReadTemperature(); //temperature reading routine using integers
float ReadTemperature_float(); //temperature reading routine using floats


#endif /* INC_HT_SENSOR_HANDLER_H_ */
