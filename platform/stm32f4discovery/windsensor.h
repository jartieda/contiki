/*
 * windsensor.h
 *
 *  Created on: 5 Oct 2013
 *      Author: Jorge
 */

#ifndef WINDSENSOR_H_
#define WINDSENSOR_H_
#include "stm32f4xx_conf.h"

#include "lib/sensors.h"

extern const struct sensors_sensor wind_sensor;

#define WIND_SENSOR "WIND"

#define WIND_SPEED 1
#define WIND_DIR   2
#define RADIATION  3





#endif /* WINDSENSOR_H_ */
