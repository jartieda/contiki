/*
 * windsensor.c
 *
 *  Created on: 5 Oct 2013
 *      Author: Jorge
 */

#include "windsensor.h"

#include "contiki.h"

const struct sensors_sensor wind_sensor;

__IO uint16_t windValue[2] = {0, 0};

/*---------------------------------------------------------------------------*/
static int
value(int type)
{
//	  uwVoltage = (uwADCxConvertedVoltage)/1000;
//	  uwMVoltage = (uwADCxConvertedVoltage%1000)/100;

	if (type== WIND_SPEED)
	{
		return windValue[1];
	}else if(type == WIND_DIR)
	{
		return windValue[0];
	}else{
		return 0;
	}
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
	  ADC_InitTypeDef       ADC_InitStructure;
	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	  DMA_InitTypeDef       DMA_InitStructure;
	  GPIO_InitTypeDef      GPIO_InitStructure;

	  /* Enable ADCx, DMA and GPIO clocks ****************************************/
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);


	  /* DMA2 Stream0 channel2 configuration **************************************/
	  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)windValue;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStructure.DMA_BufferSize = 2;
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	  DMA_Cmd(DMA2_Stream0, ENABLE);

	  /* Configure ADC3 Channel7 pin as analog input ******************************/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* ADC Common Init **********************************************************/
	  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	  ADC_CommonInit(&ADC_CommonInitStructure);

	  /* ADC1 Init ****************************************************************/
	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_NbrOfConversion = 2;
	  ADC_Init(ADC1, &ADC_InitStructure);

	  /* ADC3 regular channel7 configuration *************************************/
	  //wind dir
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
	  //wind speed
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_3Cycles);

	  /* Enable DMA request after last transfer (Single-ADC mode) */
	  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	  /* Enable ADC3 DMA */
	  ADC_DMACmd(ADC1, ENABLE);

	  /* Enable ADC3 */
	  ADC_Cmd(ADC1, ENABLE);

	  ADC_SoftwareStartConv(ADC1);

  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{

  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(wind_sensor, WIND_SENSOR,
	       value, configure, status);



