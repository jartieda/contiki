/*
 * RGBled.c
 *
 *  Created on: 16 Oct 2013
 *      Author: Jorge
 */
#include <RGBled.h>

void RGBled_init(){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t CCR1_Val = 333;
	uint16_t CCR2_Val = 249;
	uint16_t CCR3_Val = 166;
	uint16_t CCR4_Val = 83;
	uint16_t PrescalerValue = 0;

	  GPIO_InitTypeDef GPIO_InitStructure;

	  /* TIM3 clock enable */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	  /* GPIOC clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	  /* GPIOC Configuration: TIM3 CH1 (PC6), TIM3 CH2 (PC7), TIM3 CH3 (PC8) and TIM3 CH4 (PC9) */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect TIM5 pins to AF2 */
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);

	  /* -----------------------------------------------------------------------
	    TIM5 Configuration: generate 4 PWM signals with 4 different duty cycles.

	    In this example TIM5 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	    since APB1 prescaler is different from 1.
	      TIM5CLK = 2 * PCLK1
	      PCLK1 = HCLK / 4
	      => TIM5CLK = HCLK / 2 = SystemCoreClock /2

	    To get TIM3 counter clock at 21 MHz, the prescaler is computed as follows:
	       Prescaler = (TIM5CLK / TIM5 counter clock) - 1
	       Prescaler = ((SystemCoreClock /2) /21 MHz) - 1

	    To get TIM5 output clock at 30 KHz, the period (ARR)) is computed as follows:
	       ARR = (TIM5 counter clock / TIM3 output clock) - 1
	           = 665

	    TIM5 Channel1 duty cycle = (TIM5_CCR1/ TIM5_ARR)* 100 = 50%
	    TIM5 Channel2 duty cycle = (TIM5_CCR2/ TIM5_ARR)* 100 = 37.5%
	    TIM5 Channel3 duty cycle = (TIM5_CCR3/ TIM5_ARR)* 100 = 25%
	    TIM5 Channel4 duty cycle = (TIM5_CCR4/ TIM5_ARR)* 100 = 12.5%

	    Note:
	     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	     function to update SystemCoreClock variable value. Otherwise, any configuration
	     based on this variable will be incorrect.
	  ----------------------------------------------------------------------- */


	  /* Compute the prescaler value */
	  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 21000000) - 1;

	  /* Time base configuration */
	  TIM_TimeBaseStructure.TIM_Period = 665;
	  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	  /* PWM1 Mode configuration: Channel1 */
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  TIM_OC1Init(TIM5, &TIM_OCInitStructure);

	  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

	  /* PWM1 Mode configuration: Channel2 */
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

	  TIM_OC2Init(TIM5, &TIM_OCInitStructure);

	  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

	  /* PWM1 Mode configuration: Channel3 */
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

	  TIM_OC3Init(TIM5, &TIM_OCInitStructure);

	  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);


	  TIM_ARRPreloadConfig(TIM5, ENABLE);

	  /* TIM3 enable counter */
	  TIM_Cmd(TIM5, ENABLE);

}


