/*
 * dht11.c
 *
 *  Created on: 3 Oct 2013
 *      Author: Jorge
 */



/*
 * Device driver for the Sensirion SHT1x/SHT7x family of humidity and
 * temperature sensors.
 */

#include "dht11.h"

#include "contiki.h"
#include <stdio.h>
#include <signal.h>

const struct sensors_sensor DHT11_sensor;
uint8_t dht11_data[6];
unsigned long _lastreadtime;
char firstreading=1;
char *text;
static uint8_t laststate = 1;
static uint32_t counter = 0;
static uint8_t j = 0, i;

PROCESS(dht11_process, "wifi spi process");

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
#define DHT11_PORT GPIOD
#define DHT11_PIN GPIO_Pin_13

//DHT::DHT(uint8_t pin, uint8_t type) {
//  _pin = pin;
//  _type = type;
//  firstreading = true;
//}
void set_dir(uint8_t dir){
//	GPIO_InitTypeDef  GPIO_InitStructure;
//
//	// GPIOD Periph clock enable
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//
//	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
//	GPIO_InitStructure.GPIO_Pin = DHT11_PIN;
//	if (dir !=0){
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	}else{
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	}
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(DHT11_PORT, &GPIO_InitStructure);
	//0 IN
	//1 OUT
	if (dir==1){
		DHT11_PORT->MODER  &= ~(GPIO_MODER_MODER0 << (13 * 2));
		DHT11_PORT->MODER |= (((uint32_t)GPIO_Mode_OUT) << (13 * 2));
	}else{
		DHT11_PORT->MODER  &= ~(GPIO_MODER_MODER0 << (13 * 2));
		DHT11_PORT->MODER |= (((uint32_t)GPIO_Mode_IN) << (13 * 2));
	}

}
void dht11_begin(void) {
  // set up the pins!

	GPIO_InitTypeDef  GPIO_InitStructure;

	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = DHT11_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(DHT11_PORT, &GPIO_InitStructure);

//  set_dir(0);
    GPIO_SetBits(DHT11_PORT, DHT11_PIN);

  _lastreadtime = 0;
}

float dht11_convertCtoF(float c) {
	return c * 9 / 5 + 32;
}

//boolean S == Scale.  True == Farenheit; False == Celcius
/*float dht11_readTemperature(char S) {
  float f;
  if (read()) {
    switch (_type) {
    case DHT11:
      f = dht11_data[2];
      if(S)
      	f = dht11_convertCtoF(f);
      return f;
    case DHT22:
    case DHT21:
      f = dht11_data[2] & 0x7F;
      f *= 256;
      f += dht11_data[3];
      f /= 10;
      if (dht11_data[2] & 0x80)
	f *= -1;
      if(S)
	f = dht11_convertCtoF(f);

      return f;
    }
  }
  PRINTF("Read fail");
  return -99;
}*/
/*
float dht11_readHumidity(void) {
  float f;
  if (read()) {
    switch (_type) {
    case DHT11:
      f = dht11_data[0];
      return f;
    case DHT22:
    case DHT21:
      f = dht11_data[0];
      f *= 256;
      f += dht11_data[1];
      f /= 10;
      return f;
    }
  }
  PRINTF("Read fail");
  return -1;
}
*/

#define US_TIME  16
//  SystemCoreClock/1000000    /* 10 uS */
void
sleep_us (int us){
    volatile int    i;
    while (us>0) {
    	us--;
    	for (i = 0; i < US_TIME; i++) {
            ;    /* Burn cycles. */
        }
    }
}
//PT_THREAD(dht11_read(struct pt *pt)) {

// PT_BEGIN(pt);
/*---------------------------------------------------------------------------*/
  PROCESS_THREAD(dht11_process, ev, _data)
  {
	  PROCESS_BEGIN();
	  //unsigned long currenttime;
	while(1)
	{
	    static struct etimer etimer_dht11;

	    // pull the pin high and wait 250 milliseconds
		//set_dir(1);
		GPIO_SetBits(DHT11_PORT, DHT11_PIN);

		etimer_set(&etimer_dht11, CLOCK_SECOND / 4);//250ms
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etimer_dht11));

		dht11_data[0] = dht11_data[1] = dht11_data[2] = dht11_data[3] = dht11_data[4] = 0;

		// now pull it low for ~20 milliseconds
		GPIO_ResetBits(DHT11_PORT, DHT11_PIN);
		Delay(20);
	  GPIO_SetBits(DHT11_PORT, DHT11_PIN);

	  sleep_us(1);

	  // read in timings
	  uint32_t counters[85];
	  laststate = GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN);
	  j=0;
	  for ( i=0; i< MAXTIMINGS; i++) {
		//counter = clock_time();
		counter = 0;
		//while (digitalRead(_pin) == laststate) {

		while (GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN) == laststate) {
			//etimer_set(&etimer, CLOCK_SECOND / 4);
			//PT_WAIT_UNTIL(pt, (GPIOGetValue(DHT11_PORT,DHT11_PIN) != laststate)||etimer_expired(&etimer));
			counter++;
			sleep_us(1);
			if (counter >= 0xFF) {
	//			puts("couner mui grande");
				break;
			}

		}

		//    laststate = digitalRead(_pin);
		laststate = GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN);
		//counter = clock_time()-counter;//FIXME check overflow!!!!
		//if (counter == 255) break;

		// ignore first 3 transitions
		if ((i >= 4) && (i%2 == 0)) {
		  //shove each bit into the storage bytes
		  dht11_data[j/8] <<= 1;
		  counters[j]=counter;
		  if (counter > 20){
			  dht11_data[j/8] |= 1;
			  //GPIOSetValue(2,10,1 );
		  }else{
			  //GPIOSetValue(2,10,0 );
		  }
		  j++;
		}

	  }

	  // check we read 40 bits and that the checksum matches
	  if ((j >= 40) &&
		  (dht11_data[4] == ((dht11_data[0] + dht11_data[1] + dht11_data[2] + dht11_data[3]) & 0xFF)) && (dht11_data[4]!=0 )) {

		  sensors_changed(&DHT11_sensor);
	  }
	  //set_dir(1);
	  GPIO_ResetBits(DHT11_PORT, DHT11_PIN);

	  etimer_set(&etimer_dht11, CLOCK_SECOND);
	  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etimer_dht11));

  }

  PROCESS_END();

}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
	if (type== DHT11_SENSOR_HUM)
	{
		return dht11_data[0];
	}else if(type == DHT11_SENSOR_TEMP)
	{
		return dht11_data[2];
	}else{
		return 0;
	}
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
  dht11_begin();
  process_start(&dht11_process, NULL);

  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{

  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(DHT11_sensor, DHT11_SENSOR,
	       value, configure, status);


