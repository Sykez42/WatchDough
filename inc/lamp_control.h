/*
 * lamp_control.h
 *
 *  Created on: 7 ott 2023
 *      Author: merca
 */

#ifndef INC_LAMP_CONTROL_H_
#define INC_LAMP_CONTROL_H_

#include "stm32l4xx_hal.h"
#include "commons.h"

//function prototypes

void Lamp_OFF(void);
void Lamp_ON(void);
void Auto_lamp_control(float target_temp, float sensor_temp);


#endif /* INC_LAMP_CONTROL_H_ */
