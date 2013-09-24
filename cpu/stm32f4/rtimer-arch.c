/**
 * \addtogroup mb851-platform
 *
 * @{
 */

/*
 * Copyright (c) 2010, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
* \file
*			Real-timer specific implementation for STM32W.
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*/

#include "sys/energest.h"
#include "sys/rtimer.h"
#include "stm32f4xx_conf.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static uint16_t saved_TIM1CFG;
static uint32_t time_msb = 0;   /* Most significant bits of the current time. */

/* time of the next rtimer event. Initially is set to the max
    value. */
static rtimer_clock_t next_rtimer_time = 0;

/*---------------------------------------------------------------------------*/
// void
// halTimer1Isr(void)
// {
  // if(INT_TIM1FLAG & INT_TIMUIF) {
    // rtimer_clock_t now, clock_to_wait;
    // /* Overflow event. */
    // /* PRINTF("O %4x.\r\n", TIM1_CNT); */
    // /* printf("OV "); */
    // time_msb++;
    // now = ((rtimer_clock_t) time_msb << 16) | TIM1_CNT;
    // clock_to_wait = next_rtimer_time - now;

    // if(clock_to_wait <= 0x10000 && clock_to_wait > 0) { 
      // /* We must now set the Timer Compare Register. */
      // TIM1_CCR1 = (uint16_t) clock_to_wait;
      // INT_TIM1FLAG = INT_TIMCC1IF;
      // INT_TIM1CFG |= INT_TIMCC1IF;      /* Compare 1 interrupt enable. */
    // }
    // INT_TIM1FLAG = INT_TIMUIF;
  // } else {
    // if(INT_TIM1FLAG & INT_TIMCC1IF) {
      // /* Compare event. */
      // INT_TIM1CFG &= ~INT_TIMCC1IF;       /* Disable the next compare interrupt */
      // PRINTF("\nCompare event %4x\r\n", TIM1_CNT);
      // PRINTF("INT_TIM1FLAG %2x\r\n", INT_TIM1FLAG);
      // ENERGEST_ON(ENERGEST_TYPE_IRQ);
      // rtimer_run_next();
      // ENERGEST_OFF(ENERGEST_TYPE_IRQ);
      // INT_TIM1FLAG = INT_TIMCC1IF;
    // }
  // }
// }

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
   uint16_t capture = 0;

  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

    /* LED1 toggling with frequency = 73.24 Hz */
	capture = TIM_GetCapture1(TIM3);
//    TIM_SetCompare1(TIM3, capture + CCR1_Val);

	PRINTF("\nCompare event %4x\r\n", TIM1_CNT);
    PRINTF("INT_TIM1FLAG %2x\r\n", INT_TIM1FLAG);
    ENERGEST_ON(ENERGEST_TYPE_IRQ);
    rtimer_run_next();
    ENERGEST_OFF(ENERGEST_TYPE_IRQ);
  }
  else
  {
  rtimer_clock_t now, clock_to_wait;
    /* Overflow event. */
    /* PRINTF("O %4x.\r\n", TIM1_CNT); */
    /* printf("OV "); */
    time_msb++;
    now = (rtimer_clock_t)TIM_GetCapture1(TIM3); //((rtimer_clock_t) time_msb << 16) | TIM1_CNT;
    clock_to_wait = next_rtimer_time - now;

    if(clock_to_wait <= 0x10000 && clock_to_wait > 0) { 
      /* We must now set the Timer Compare Register. */
      //TIM1_CCR1 = (uint16_t) clock_to_wait;
  	capture = TIM_GetCapture1(TIM3);
    TIM_SetCompare1(TIM3, capture + clock_to_wait);

	  /* TIM Interrupts enable */
	  TIM_ITConfig(TIM3, TIM_IT_CC1 , ENABLE);

      //INT_TIM1FLAG = INT_TIMCC1IF;
      //INT_TIM1CFG |= INT_TIMCC1IF;      /* Compare 1 interrupt enable. */
    }
    //INT_TIM1FLAG = INT_TIMUIF;
  }
}

/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
	uint16_t PrescalerValue = 0;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  
  /* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM3 counter clock at 6 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /6 MHz) - 1
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = 146.48 Hz
    ==> Toggling frequency = 73.24 Hz
    
    C2 update rate = TIM3 counter clock / CCR2_Val = 219.7 Hz
    ==> Toggling frequency = 109.8 Hz
    
    CC3 update rate = TIM3 counter clock / CCR3_Val = 439.4 Hz
    ==> Toggling frequency = 219.7 Hz
    
    CC4 update rate = TIM3 counter clock / CCR4_Val = 878.9 Hz
    ==> Toggling frequency = 439.4 Hz

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */   


  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 6000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1000;//CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
  
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_CC1 , ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

  // TIM1_CR1 = 0;
  // TIM1_PSC = RTIMER_ARCH_PRESCALER;

  // /* Counting from 0 to the maximum value. */
  // TIM1_ARR = 0xffff;

  // /* Bits of TIMx_CCMR1 as default. */
  // /* Update Generation. */
  // TIM1_EGR = TIM_UG;
  // INT_TIM1FLAG = 0xffff;

  // /* Update interrupt enable (interrupt on overflow).*/
  // INT_TIM1CFG = INT_TIMUIF;

  // /* Counter enable. */
  // TIM1_CR1 = TIM_CEN;

  // /* Enable top level interrupt. */
  // INT_CFGSET = INT_TIM1;

}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_disable_irq(void)
{
  TIM_ITConfig(TIM3, TIM_IT_CC1 , DISABLE);
  TIM_Cmd(TIM3, DISABLE);
  //ATOMIC(saved_TIM1CFG = INT_TIM1CFG; INT_TIM1CFG = 0;)
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_enable_irq(void)
{
  TIM_ITConfig(TIM3, TIM_IT_CC1 , ENABLE);
  TIM_Cmd(TIM3, ENABLE);
 // INT_TIM1CFG = saved_TIM1CFG;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_now(void)
{
  rtimer_clock_t t;

//  ATOMIC(t = ((rtimer_clock_t) time_msb << 16) | TIM1_CNT;)
  t = (rtimer_clock_t) TIM_GetCapture1(TIM3);
  return t;
}

/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  rtimer_clock_t now, clock_to_wait;
  PRINTF("rtimer_arch_schedule time %4x\r\n", /*((uint32_t*)&t)+1, */ 
         (uint32_t)t);
  next_rtimer_time = t;
  now = rtimer_arch_now();
  clock_to_wait = t - now;

  PRINTF("now %2x\r\n", TIM1_CNT);
  PRINTF("clock_to_wait %4x\r\n", clock_to_wait);

  if(clock_to_wait <= 0x10000) {
  /* We must now set the Timer Compare Register. */
//    TIM1_CCR1 = (uint16_t)now + (uint16_t)clock_to_wait;
//    INT_TIM1FLAG = INT_TIMCC1IF;
//    INT_TIM1CFG |= INT_TIMCC1IF;        /* Compare 1 interrupt enable. */
//    PRINTF("2-INT_TIM1FLAG %2x\r\n", INT_TIM1FLAG);
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = (uint16_t)now + (uint16_t)clock_to_wait;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
  
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_CC1 , ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

  }
  /* else compare register will be set at overflow interrupt closer to
     the rtimer event. */
}
/*---------------------------------------------------------------------------*/
/** @} */
