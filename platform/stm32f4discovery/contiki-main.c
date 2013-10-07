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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "CC3000/wlan.h"
#include "CC3000/spi.h"
#include "dht11.h"
//#include "dcmi_ov9655.h"
#include "dcmi_ov2640.h"
#include "socket.h"

PROCINIT(&etimer_process );

SENSORS(&DHT11_sensor);

//OV9655_IDTypeDef  OV9655_Camera_ID;
OV2640_IDTypeDef  OV2640_Camera_ID;

extern Camera_TypeDef       Camera;
extern ImageFormat_TypeDef  ImageFormat;
extern __IO uint8_t         ValueMax;
extern const uint8_t *      ImageForematArray[];

uint32_t clocktime;

//#define WIFI_BOARD
#define CAMERA_BOARD
//#define DISCOVERY_BOARD


// Private function prototypes
void Delay(volatile uint32_t nCount);
void init();
extern uint8_t frame_buffer[160*120];

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

  int i = 0;
  for (i = 0 ; i<19200; i++)
      frame_buffer[i]=0;

  /* Initializes the DCMI interface (I2C and GPIO) used to configure the camera */
  OV2640_HW_Init();
  Delay(5);/*necessary for init camera */
  /* Read the OV9655/OV2640 Manufacturer identifier */
  OV2640_ReadID(&OV2640_Camera_ID);

  if(OV2640_Camera_ID.PIDH  == 0x26)
  {
	Camera = OV2640_CAMERA;
	ValueMax = 2;
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
    OV2640_Init(BMP_QQVGA);
    OV2640_QQVGAConfig();
    OV2640_BandWConfig(0x18);

    /* Enable DMA2 stream 1 and DCMI interface then start image capture */
    DMA_Cmd(DMA2_Stream1, ENABLE);
    DCMI_Cmd(ENABLE);

    /* Insert 100ms delay: wait 100ms */
    Delay(200);

    DCMI_CaptureCmd(ENABLE);
  }

  //watchdog_start();

  /* Initializate the WIFI */
  wlan_start(0);
  wlan_connect (WLAN_SEC_WPA2 ,"wifi1",5,NULL,"smallsignals",12);

//  process_start(&wifi_spi_process, NULL);

  process_start(&sensors_process,NULL);

  int sd = -1;
  int connected = -1;
  sockaddr addr;
  unsigned short port = 80;
  char wifi_buff[] = "POST /index.php HTTP/1.1\
Host: 192.168.1.2\
User-Agent: my custom client v.1\
Content-Type: application/octet-stream\
Content-Length: 10\
\
1234567890\n";
  int wifi_buff_leng = sizeof("POST /index.php HTTP/1.1\
Host: 192.168.1.2\
User-Agent: my custom client v.1\
Content-Type: application/octet-stream\
Content-Length: 10\
\
1234567890\n");
/*  memcpy(wifi_buff, "POST /index.php HTTP/1.1\
Host: 192.168.1.2\
User-Agent: my custom client v.1\
Content-Type: application/octet-stream\
Content-Length: 10\
\
1234567890\n", wifi_buff_leng);*/
  addr.sa_family = AF_INET;
  // port
  addr.sa_data[0] = (port & 0xFF00) >> 8;
  addr.sa_data[1] = (port & 0x00FF);
  //ip
  addr.sa_data[5] = 192;
  addr.sa_data[4] = 168;
  addr.sa_data[3] = 1;
  addr.sa_data[2] = 2;
//  addr.sin_zero
  sd = socket(AF_INET,SOCK_STREAM, IPPROTO_TCP);
  connected = connect(sd,&addr,sizeof(addr));
  if (connected == 0)
  {
	  GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	  send(sd, wifi_buff, wifi_buff_leng, 0);
  }
  while(1) {
//    wlan_start(0,&wlan_pt);
    do {
		if (clocktime!=clock_seconds()) {
			clocktime=clock_seconds();
	    	if (clocktime%2==0){
	    		GPIO_SetBits(GPIOD, GPIO_Pin_14);
	    	}else{
	    		GPIO_ResetBits(GPIOD, GPIO_Pin_14);

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
	DAC_InitTypeDef  DAC_InitStructure;

	// DEBUG LEDS
	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    // Configure PD12, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_12);

	// State Leds Clock (only for Wifi BOARD)
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	// State Leds
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_12;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOE, &GPIO_InitStructure);


	// ---------- DAC ---------- //

	// Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Configuration
	/*DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);*/

	// IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Enable DAC Channel1
	DAC_Cmd(DAC_Channel_1, ENABLE);

	// Set DAC Channel1 DHR12L register
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);


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



