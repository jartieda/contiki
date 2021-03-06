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
#include "windsensor.h"
//#include "dcmi_ov9655.h"
#include "dcmi_ov2640.h"
#include "socket.h"
#include "leds.h"
#include "pressure.h"

#define GPIO_HIGH(a,b) 		a->BSRRL = b
#define GPIO_LOW(a,b)			a->BSRRH = b
#define GPIO_TOGGLE(a,b) 	a->ODR ^= b

extern int e_sprintf(char *out, const char *format, ...);

PROCINIT(&etimer_process );

SENSORS(&DHT11_sensor, &wind_sensor, &PRESSURE_sensor);

PROCESS(wifi_client_process, "wifi_client_process");

//OV9655_IDTypeDef  OV9655_Camera_ID;
OV2640_IDTypeDef  OV2640_Camera_ID;

extern Camera_TypeDef       Camera;
extern ImageFormat_TypeDef  ImageFormat;
extern __IO uint8_t         ValueMax;
extern const uint8_t *      ImageForematArray[];

extern uint8_t wifi_dhcp ;
extern uint8_t freebuff ;

uint32_t clocktime;

//#define WIFI_BOARD
#define CAMERA_BOARD
//#define DISCOVERY_BOARD
volatile int servo_angle=0; // +/- 90 degrees, use floats for fractional

void TIM1_UP_TIM10_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
  {

    // minimum high of 600 us for -90 degrees, with +90 degrees at 2400 us, 10 us per degree
    //  timer timebase set to us units to simplify the configuration/math
	  GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	  //GPIO_ToggleBits(GPIOE, GPIO_Pin_9);

    TIM1->CCR1 = 600 + ((servo_angle + 90) * 10); // where angle is an int -90 to +90 degrees, PC.6
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  }
}
void SERVO_Configuration(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* TIM1 channel 2 pin (PE.9) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

    NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	/* Enable the TIM1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    // Simplify the math here so TIM_Pulse has 1 us units

    // Use (24-1) for VL @ 24 MHz
    // Use (72-1) for STM32 @ 72 MHz

    TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1;  // 24 MHz / 24 = 1 MHz
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20 ms)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    // Set up 4 channel servo
    TIM_OCStructInit (& TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 600 + 900; // 1500 us - Servo Top Centre
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);    // Channel 1 configuration = PC.06 TIM3_CH1

    // PWM1 Mode configuration: Channel1
    // Edge -aligned; not single pulse mode
    TIM_OCStructInit (& TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM1 , &TIM_OCInitStructure);

    TIM_BDTRInitTypeDef bdtr;
    TIM_BDTRStructInit(&bdtr);
    bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &bdtr);


    // Enable Timer Interrupt and Timer
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //
    TIM_Cmd(TIM1 , ENABLE);

    TIM_SetCompare1(TIM1, 1500);    // 2370 = 1.5ms - for Servo

}
// Private function prototypes
void Delay(volatile uint32_t nCount);
void init();
extern uint8_t frame_buffer[160*120];//FIXME ESTO DEBERIA SER DE 2*160*120 O uint16_t

unsigned int idle_count = 0;

char capture_next_frame= 0;

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

  SERVO_Configuration();

  int i = 0;
  for (i = 0 ; i<19200; i++)
      frame_buffer[i]=i%256;

  /* Initializes the DCMI interface (I2C and GPIO) used to configure the camera */
  OV2640_HW_Init();
  Delay(5);/*necessary for init camera */
  /* Read the OV9655/OV2640 Manufacturer identifier */
  OV2640_ReadID(&OV2640_Camera_ID);
  GPIO_SetBits(GPIOD, GPIO_Pin_12);

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


  /* Initializate the WIFI */
  wlan_start(0);
//  wlan_connect (WLAN_SEC_WPA2 ,"wifi1",5,NULL,"smallsignals",12);
  wlan_connect (WLAN_SEC_WEP ,"WLAN_75",7,NULL,"Z0002CFA92E75",13);

  while(wifi_dhcp == 0){
	  GPIO_SetBits(GPIOD, GPIO_Pin_12);
	  Delay(300);
	  GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	  Delay(300);
  }

  process_start(&sensors_process,NULL);

  process_start(&wifi_client_process, NULL);
  watchdog_start();
  GPIO_ResetBits(GPIOD, GPIO_Pin_12);

  while(1) {
    do {
		if (clocktime!=clock_seconds()) {
			clocktime=clock_seconds();
	    	if (clocktime%2==0){
	    		GPIO_SetBits(GPIOD, GPIO_Pin_15);
	    		servo_angle=-45;
	    	}else{
	    		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	    		servo_angle=45;
	    	}
		}
    	watchdog_periodic();
		etimer_request_poll();
		process_poll(&wifi_client_process);
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
  char wifi_buff[] = "POST /nowcasting/recibe_foto_small.php HTTP/1.1\n\
Host: 80.28.200.188\n\
User-Agent: my custom client v.1\n\
Content-Type: application/octet-stream\n\
Content-Length: 38400\n\
\n";
  int wifi_buff_leng = sizeof("POST /nowcasting/recibe_foto_small.php HTTP/1.1\n\
Host: 80.28.200.188\n\
User-Agent: my custom client v.1\n\
Content-Type: application/octet-stream\n\
Content-Length: 38400\n\
\n");

  char sensor_buff[400] = "POST /nowcasting/?r=meteoVal/addfromstation HTTP/1.1\n\
Host: 80.28.200.188\n\
User-Agent: my custom client v.1\n\
Content-Type: application/x-www-form-urlencoded\n\
Content-Length: ";
  int sensor_buff_leng= sizeof("POST /nowcasting/?r=meteoVal/addfromstation HTTP/1.1\n\
Host: 80.28.200.188\n\
User-Agent: my custom client v.1\n\
Content-Type: application/x-www-form-urlencoded\n\
Content-Length: ");
  char content_length[10];
  int conten_length_leng=0;
  char sensor_payload[200];
  int sensor_payload_leng = 0;
/*Windspeed
Winddirection
temp
pres
pira
hum
*/

  PROCESS_BEGIN();
  sd = -1;
  connected = -1;
  addr.sa_family = AF_INET;
//  // port
//  port = 81;
//  addr.sa_data[0] = (port & 0xFF00) >> 8;
//  addr.sa_data[1] = (port & 0x00FF);
//  //ip
//  addr.sa_data[2] = 192;
//  addr.sa_data[3] = 168;
//  addr.sa_data[4] = 1;
//  addr.sa_data[5] = 2;
  // port
  port = 80;
  addr.sa_data[0] = (port & 0xFF00) >> 8;
  addr.sa_data[1] = (port & 0x00FF);
  //ip
  addr.sa_data[2] = 80;
  addr.sa_data[3] = 28;
  addr.sa_data[4] = 200;
  addr.sa_data[5] = 188;

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
              //etimer_set(&timer_send_packet, CLOCK_SECOND * 3);
              //image divided in 16 packets of 1200: 16 = 160 *120 /1200
              //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));
              PROCESS_YIELD_UNTIL(tSLInformation.usNumberOfFreeBuffers>5);
			  //freebuff=0;
              //
              for (ipkg = 0; ipkg < 32; ipkg++)
              {
            	  GPIO_TOGGLE(GPIOD, GPIO_Pin_14);
                  send(sd, frame_buffer+(ipkg*1200),1200, 0);
                  //etimer_reset(&timer_send_packet);
                  watchdog_periodic();
                  PROCESS_YIELD_UNTIL(tSLInformation.usNumberOfFreeBuffers>5);
                  freebuff=0;

              }
          }else{
        	  while(1);//make watchdog reset de device
          }
//          PROCESS_WAIT_EVENT_UNTIL(freebuff==1);

          //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));
          GPIO_ResetBits(GPIOD, GPIO_Pin_14);
          watchdog_periodic();
          closesocket(sd);
          watchdog_periodic();
      }else{
    	  while(1);//make watchdog reset de device
      }
      GPIO_SetBits(GPIOD, GPIO_Pin_14);
      // start sending measure
      etimer_set(&timer_send_image, CLOCK_SECOND * 15);
      etimer_reset(&timer_send_image);//deberian ser 15 segundos
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_image));
      GPIO_ResetBits(GPIOD, GPIO_Pin_14);
      GPIO_SetBits(GPIOD, GPIO_Pin_12);
      watchdog_periodic();
      sd = socket(AF_INET,SOCK_STREAM, IPPROTO_TCP);
      if (sd != -1)
      {
          watchdog_periodic();
          connected = connect(sd,&addr,sizeof(addr));
          if (connected == 0)
          {
        	  watchdog_periodic();
              GPIO_SetBits(GPIOD, GPIO_Pin_14);
              //header
              int rad = wind_sensor.value(RADIATION);
              int wind= wind_sensor.value(WIND_SPEED);
              int wdir= wind_sensor.value(WIND_DIR);
              int hum = DHT11_sensor.value(DHT11_SENSOR_HUM);
              int temp = DHT11_sensor.value(DHT11_SENSOR_TEMP);
              /*char s_rad[5];
              itoa(rad, s_rad);
              char s_win[5];
              itoa(wind, s_win);
              char s_dir[5];
			  itoa(wdir, s_dir);*/
              sensor_payload_leng= 72;
              /*itoa(sensor_payload_leng, content_length );
              strcpy(&sensor_buff[196],content_length);
              strcpy(&sensor_buff[200],"\n\n");
              strcpy(&sensor_buff[200])*/
              e_sprintf(&sensor_buff[170], "%d\n\
\n\
Windspeed=%03d&Winddirection=%03d&pira=%03d&temp=%03d&hum=%03d&pres=%04d\n",
                   sensor_payload_leng, wind,wdir,rad, temp,hum, PRESSURE_decPcomp);
              send(sd, sensor_buff, strlen(sensor_buff)	, 0);
              etimer_set(&timer_send_packet, CLOCK_SECOND * 2);
              PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));
              PROCESS_YIELD_UNTIL(tSLInformation.usNumberOfFreeBuffers>5);
          }
          closesocket(sd);
      }

      //GPIO_ResetBits(GPIOD, GPIO_Pin_12); reset in dcmi handler
      capture_next_frame=1;

      etimer_reset(&timer_send_image);
  }
  PROCESS_END();
}
void DCMI_IRQHandler(void)
{
	if(DCMI_GetITStatus(DCMI_IT_FRAME)	!= RESET)
	{
		if (capture_next_frame == 1){
		      DMA_Cmd(DMA2_Stream1, ENABLE);
		      //DCMI_Cmd(ENABLE);
		      /* Insert 100ms delay: wait 100ms */
		      //Delay(200);
		      //DCMI_CaptureCmd(ENABLE);
		      DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		      GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		      capture_next_frame=0;
		}
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);
	}
}

