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
#include "CC3000/nvmem.h"
#include "CC3000/security.h"
#include "socket.h"
#include "RGBled.h"
#include "ssd1306.h"

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


  //watchdog_start();

  ssd1306Init();
  int kk=1;
  while (1){
	  for (int jj = 0; jj<128; jj++){
		  for (int ii =0; ii<64;ii++){
			  if (jj%kk ==0) ssd1306ClearPixel(jj,ii);
			  else 		     ssd1306DrawPixel(jj, ii);
		  }
	  }
	  kk++;
	  if (kk > 30) kk = 1;
	  ssd1306Refresh();
  }
  /* Initializate the WIFI */
  wlan_start(0);
  unsigned char patchVer[10];
  nvmem_read_sp_version(patchVer);
  if (! GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) )
  {
	  nvmem_create_entry(NVMEM_AES128_KEY_FILEID, AES128_KEY_SIZE);
	  //wlan_first_time_config_start();
	  aes_write_key("1234567890123456");
	  wlan_smart_config_set_prefix("TTT");
	  wlan_smart_config_start(0);

		GPIO_SetBits(GPIOE, GPIO_Pin_10);

  }else{
	  //wlan_connect (WLAN_SEC_WPA2 ,"MOVISTAR_E22F",13,NULL,"EvtRYfsVPxtLfsNm2TLP",20);
  }
  while(wifi_dhcp == 0){
	  Delay(100);
  }
	GPIO_ResetBits(GPIOE, GPIO_Pin_10);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);

  autostart_start(autostart_processes);

  process_start(&wifi_client_process, NULL);
  int brightness = 0;
  while(1) {
    do {
		if (clocktime!=clock_seconds()) {
			clocktime=clock_seconds();

	          brightness+=10;

	    	if (clocktime%2==0){
	    		GPIO_SetBits(GPIOE, GPIO_Pin_10);
//	    		GPIO_SetBits(GPIOE, GPIO_Pin_11);
	    		GPIO_SetBits(GPIOE, GPIO_Pin_12);
	    	}else{
	    		GPIO_ResetBits(GPIOE, GPIO_Pin_10);
//	    		GPIO_ResetBits(GPIOE, GPIO_Pin_11);
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

	GPIO_InitTypeDef   GPIO_InitStructure2;

	/* Enable GPIOA clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PA0 pin as input floating */
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure2);

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
  char wifi_buff[] = "POST /moodlight/get_color.php HTTP/1.1\n\
Host: 192.168.1.2\n\
User-Agent: my custom client v.1\n\
Content-Type: application/octet-stream\n\
Content-Length: 0\n\
\n";
  int wifi_buff_leng = sizeof("POST /moodlight/get_color.php HTTP/1.1\n\
Host: 192.168.1.2\n\
User-Agent: my custom client v.1\n\
Content-Type: application/octet-stream\n\
Content-Length: 0\n\
\n");
  char recv_buff[400];
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
	    	  GPIO_SetBits(GPIOE, GPIO_Pin_11);
	    	  //header
              send(sd, wifi_buff, wifi_buff_leng, 0);
              etimer_set(&timer_send_packet, CLOCK_SECOND * 5);
              PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));
              //image divided in 16 packets of 1200: 16 = 160 *120 /1200
//              for (ipkg = 0; ipkg < 32; ipkg++)
//              {
//                  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));
//       //         send(sd, frame_buffer+(ipkg*1200),1200, 0);
//                  etimer_reset(&timer_send_packet);
//              }

              long n_received = recv(sd, recv_buff, 400, 0);
              char b[3]={48,48,48};
              char g[3]={48,48,48};
              char r[3]={48,48,48};
              int n = 2;
              n_received--;
              while (recv_buff[n_received]!=' '){
            	  b[n]=recv_buff[n_received];
            	  n--;
            	  n_received--;
              }
              n_received--;
              n = 2;
              while (recv_buff[n_received]!=' '){
				  g[n]=recv_buff[n_received];
				  n--;
				  n_received--;
			  }
              n_received--;
              n = 2;
			  while (recv_buff[n_received]>=48&&recv_buff[n_received]<=57){
			    r[n]=recv_buff[n_received];
			    n--;
			    n_received--;
			  }
			  int rr = (r[0]-48)*100+(r[1]-48)*10+(r[2]-48);
			  int gg = (g[0]-48)*100+(g[1]-48)*10+(g[2]-48);
			  int bb = (b[0]-48)*100+(b[1]-48)*10+(b[2]-48);
			  TIM5->CCR1 = rr*2; // set brightness
			  TIM5->CCR3 = gg*2; // set brightness
			  TIM5->CCR2 = bb*2; // set brightness

              etimer_reset(&timer_send_packet);
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


