/*
 * dht11.h
 *
 *  Created on: 3 Oct 2013
 *      Author: Jorge
 */

#ifndef DHT11_H_
#define DHT11_H_

#include "stm32f4xx_conf.h"

#include "lib/sensors.h"

extern const struct sensors_sensor DHT11_sensor;

#define DHT11_SENSOR "DHT11"

//void button_press(void);
#define DHT11_SENSOR_TEMP 1
#define DHT11_SENSOR_HUM 2


// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

uint8_t _pin, _type;
char dht11_read(struct pt *pt);
extern struct pt dht11_pt;

void dht11_init(uint8_t pin, uint8_t type);
void dht11_begin(void);
float dht11_readTemperature(char S);
float dht11_convertCtoF(float);
float dht11_readHumidity(void);



#endif /* DHT11_H_ */
