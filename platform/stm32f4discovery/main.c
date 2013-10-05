#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "CC3000/wlan.h"
#include "CC3000/spi.h"
//#include "dcmi_ov9655.h"
#include "dcmi_ov2640.h"

//OV9655_IDTypeDef  OV9655_Camera_ID;
OV2640_IDTypeDef  OV2640_Camera_ID;

extern Camera_TypeDef       Camera;
extern ImageFormat_TypeDef  ImageFormat;
extern __IO uint8_t         ValueMax;
extern const uint8_t *      ImageForematArray[];

//#define WIFI_BOARD
#define CAMERA_BOARD
//#define DISCOVERY_BOARD

// Private variables
volatile uint32_t time_var1, time_var2;

// Private function prototypes
void Delay(volatile uint32_t nCount);
void init();
void calculation_test();
void dac_test();
extern uint8_t frame_buffer[160*120];

int main(void) {
	init();
	int i = 0;
	for (i = 0 ; i<19200; i++)
		frame_buffer[i]=0;

	  /* Initializes the DCMI interface (I2C and GPIO) used to configure the camera */
	  OV2640_HW_Init();
	  Delay(5);
	  /* Read the OV9655/OV2640 Manufacturer identifier */
	  //OV9655_ReadID(&OV9655_Camera_ID);

		  OV2640_ReadID(&OV2640_Camera_ID);

		if(OV2640_Camera_ID.PIDH  == 0x26)
		  {
			Camera = OV2640_CAMERA;
			ValueMax = 2;
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
  		    OV2640_Init(BMP_QQVGA);
	        OV2640_QQVGAConfig();
	        OV2640_BandWConfig(0x18);
		  }

	  /* Enable DMA2 stream 1 and DCMI interface then start image capture */
	  DMA_Cmd(DMA2_Stream1, ENABLE);
	  DCMI_Cmd(ENABLE);

	  /* Insert 100ms delay: wait 100ms */
	  Delay(200);

	  DCMI_CaptureCmd(ENABLE);

 	wlan_start(0);
	calculation_test();
	//dac_test();

	for(;;) {

	}

	return 0;
}

void dac_test() {
	for(;;) {
		for (float i = 0;i < 2.0 * M_PI;i += (2.0 * M_PI) / 100.0) {
			DAC_SetChannel1Data(DAC_Align_12b_R, (uint32_t)((sinf(i) + 1.0) * 2047.0));
		}
	}
}

void calculation_test() {
	//float a = 1.001;
	int iteration = 0;
//	wlan_start(0);

	for(;;) {
		#ifdef WIFI_BOARD
		//WIFI Leds
		GPIO_SetBits(GPIOE, GPIO_Pin_10);
		GPIO_SetBits(GPIOE, GPIO_Pin_11);
//		GPIO_SetBits(GPIOE, GPIO_Pin_12);
		Delay(500);
		GPIO_ResetBits(GPIOE, GPIO_Pin_10);
		GPIO_ResetBits(GPIOE, GPIO_Pin_11);
		GPIO_ResetBits(GPIOE, GPIO_Pin_12);
		Delay(500);
		#endif
		#ifdef CAMERA_BOARD
		//Camera Leds
		//GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		Delay(500);
		//GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		Delay(500);
		#endif
		#ifdef DISCOVERY_BOARD
		//Discovery Leds
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		Delay(500);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		Delay(500);
		#endif
		time_var2 = 0;
		
		//for (int i = 0;i < 1;i++) {
		//	a += 0.01 * a;//sqrtf(a);
		//}
/*
		printf("Time:      %lu\n", time_var2);
		printf("Iteration: %i\n", iteration);
		printf("Value:     %lu\n", (unsigned long)a);
//		printf("Value F:   %.5f\n", -a);
		printf("Value F2:  %s\n\n", ftostr(-a, 5));
*/
		iteration++;
	}
}

void init() {
	GPIO_InitTypeDef  GPIO_InitStructure;
	//USART_InitTypeDef USART_InitStructure;
	DAC_InitTypeDef  DAC_InitStructure;

	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_SetBits(GPIOD, GPIO_Pin_13);

	// WIFI Leds Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	// WIFI Leds
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// ------ UART ------ //

	// Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	// Conf
/*	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	// Enable
	USART_Cmd(USART2, ENABLE);
*/

	// ---------- DAC ---------- //

	// Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Configuration
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	// IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Enable DAC Channel1
	DAC_Cmd(DAC_Channel_1, ENABLE);

	// Set DAC Channel1 DHR12L register
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);

	// Configure EXTI Line8 (connected to PD8 pin) in interrupt mode
	EXTILine_Config();

	// Configure wifi_pwr_en on PD9
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_9);

	// Initialize WiFi
	wlan_init(fWlanCB,0,0,0,
			fWlanReadInteruptPin,
			fWlanInterruptEnable,
			fWlanInterruptDisable,
			fWriteWlanPin);

	// Generate software interrupt: simulate a falling edge applied on  EXTI0 line
	EXTI_GenerateSWInterrupt(EXTI_Line0);

}

/*
 * Called from systick handler
 */
void timing_handler() {
	if (time_var1) {
		time_var1--;
	}

	time_var2++;
}

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nCount) {
	time_var1 = nCount;
	while(time_var1){};
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {

}

