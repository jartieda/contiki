/**
 * \addtogroup stm32f4
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
*			Leds.
* \author
*			Jorge Artieda
*/

//#include PLATFORM_HEADER
//#include BOARD_HEADER
#include "contiki-conf.h"
#include "dev/leds.h"
#include "stm32f4xx_conf.h"

//#include "hal/micro/micro-common.h"
//#include "hal/micro/cortexm3/micro-common.h"

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Configure PD12, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_12)| GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_15)| GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_14);

}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
	if (leds & LEDS_RED){
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
	}else{
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);	
	}
	if (leds & LEDS_GREEN){
		GPIO_SetBits(GPIOD, GPIO_Pin_15);	
	}else{
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);	
	}
	if (leds & LEDS_BLUE){
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
	}else{
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);	
	}
	
}
/*---------------------------------------------------------------------------*/
/** @} */
