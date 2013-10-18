#include "stm32f4xx_conf.h"
//#include <nvic.h>
#include "contiki.h"

#include <stdint.h>
#include <stdio.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>
#include <watchdog.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "CC3000/wlan.h"
#include "CC3000/spi.h"
#include "socket.h"
#include "RGBled.h"


PROCINIT(&etimer_process );


PROCESS(wifi_client_process, "wifi_client_process");


extern uint8_t wifi_dhcp ;

uint32_t clocktime;

#define WIFI_BOARD
//#define CAMERA_BOARD
//#define DISCOVERY_BOARD


// Private function prototypes
void Delay(volatile uint32_t nCount);
void init();

unsigned int idle_count = 0;


int
main()
{
//  dbg_setup_uart();
//  printf("Initialising\n");
  
  clock_init();
  watchdog_init();
  init();
  process_init();
  process_start(&etimer_process, NULL);
  autostart_start(autostart_processes);


  //watchdog_start();

  /* Initializate the WIFI */
  wlan_start(0);
  wlan_connect (WLAN_SEC_WPA2 ,"wifi1",5,NULL,"smallsignals",12);

  while(wifi_dhcp == 0){
	  Delay(100);
  }

  //process_start(&wifi_client_process, NULL);
  int brightness = 0;
  while(1) {
    do {
		if (clocktime!=clock_seconds()) {
			clocktime=clock_seconds();

	          brightness+=10;


	          TIM5->CCR1 = 333 - (brightness + 0) % 666; // set brightness
	          TIM5->CCR2 = 333 - (brightness + 166/2) % 666; // set brightness
	          TIM5->CCR3 = 333 - (brightness + 333/2) % 666; // set brightness

	    	if (clocktime%2==0){
	    		GPIO_SetBits(GPIOE, GPIO_Pin_10);
	    		GPIO_SetBits(GPIOE, GPIO_Pin_11);
	    		GPIO_SetBits(GPIOE, GPIO_Pin_12);
	    	}else{
	    		GPIO_ResetBits(GPIOE, GPIO_Pin_10);
	    		GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	    		GPIO_ResetBits(GPIOE, GPIO_Pin_12);

	    	}

		}
   // 	watchdog_periodic();
		etimer_request_poll();

    } while(process_run() > 0);
    idle_count++;
    /* Idle! */
    /* Stop processor clock */
    /* asm("wfi"::); */ 
  }
  return 0;
}


void init() {
	GPIO_InitTypeDef  GPIO_InitStructure;
	//USART_InitTypeDef USART_InitStructure;

	// DEBUG LEDS
	// GPIOD Periph clock enable

	// State Leds Clock (only for Wifi BOARD)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	// State Leds
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// State Leds Clock (only for Wifi BOARD)
	/* Enable WKUP pin  */
	PWR_WakeUpPinCmd(DISABLE);

	RGBled_init();
	/*RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// State Leds
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	//GPIO_SetBits(GPIOA, GPIO_Pin_1);
	//GPIO_SetBits(GPIOA, GPIO_Pin_2);
	 */
	// Initialize WiFi
	wlan_init(fWlanCB,0,0,0,
			fWlanReadInteruptPin,
			fWlanInterruptEnable,
			fWlanInterruptDisable,
			fWriteWlanPin);

}



/*
 * Dummy function to avoid compiler error
 */
void _init() {

}

PROCESS_THREAD(wifi_client_process, ev, data)
{
  static struct etimer timer_send_image;
  static struct etimer timer_send_packet;
  static int sd;
  static int connected;
  static sockaddr addr;
  static unsigned short port;
  static int ipkg;
  char wifi_buff[] = "POST /recibeimg.php HTTP/1.1\n\
Host: 192.168.1.2\n\
User-Agent: my custom client v.1\n\
Content-Type: application/octet-stream\n\
Content-Length: 38400\n\
\n";
  int wifi_buff_leng = sizeof("POST /recibeimg.php HTTP/1.1\n\
Host: 192.168.1.2\n\
User-Agent: my custom client v.1\n\
Content-Type: application/octet-stream\n\
Content-Length: 38400\n\
\n");

  PROCESS_BEGIN();
  sd = -1;
  connected = -1;
  addr.sa_family = AF_INET;
  // port
  port = 81;
  addr.sa_data[0] = (port & 0xFF00) >> 8;
  addr.sa_data[1] = (port & 0x00FF);
  //ip
  addr.sa_data[2] = 192;
  addr.sa_data[3] = 168;
  addr.sa_data[4] = 1;
  addr.sa_data[5] = 2;

  while (1)
  {
      etimer_set(&timer_send_image, CLOCK_SECOND * 15);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_image));
      sd = socket(AF_INET,SOCK_STREAM, IPPROTO_TCP);
      if (sd != -1)
      {
          connected = connect(sd,&addr,sizeof(addr));
          if (connected == 0)
          {
              GPIO_SetBits(GPIOD, GPIO_Pin_14);
              //header
              send(sd, wifi_buff, wifi_buff_leng, 0);
              etimer_set(&timer_send_packet, CLOCK_SECOND * 2);
              //image divided in 16 packets of 1200: 16 = 160 *120 /1200
              for (ipkg = 0; ipkg < 32; ipkg++)
              {
                  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));
                  send(sd, frame_buffer+(ipkg*1200),1200, 0);
                  etimer_reset(&timer_send_packet);
              }
          }
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));

          closesocket(sd);
      }
      GPIO_ResetBits(GPIOE, GPIO_Pin_11);
      //GPIO_SetBits(GPIOE, GPIO_Pin_12);
      //GPIO_ResetBits(GPIOE, GPIO_Pin_12);

      etimer_reset(&timer_send_image);
  }
  PROCESS_END();
}


