#include "spi.h"
#include <stdint.h>
#include <stm32f4xx_spi.h>
#include "stm32f4xx_conf.h"
#include <contiki.h>

/*---------------------------------------------------------------------------*/
PROCESS(wifi_spi_process, "wifi spi process");

#define SPI_BUFFER_SIZE         1700


uint8_t wlan_tx_buffer[SPI_BUFFER_SIZE];
uint8_t wlan_rx_buffer[SPI_BUFFER_SIZE];

void (*_pfRxHandler)();
/* The SpiOpen function is called from the wlan_start API function. The main  */
/* purpose of the function is to register the callback for the RX path,       */
/* allocate buffers for the SPI Transmission and Reception of data, and       */
/* initialize the SPI internal state-machine.                                 */
void SpiOpen(void (*pfRxHandler))
{
	_pfRxHandler = pfRxHandler;

	GPIO_InitTypeDef GPIO_InitStructure;

	/*!< Enable the SPI clock */
	WIFI_SPI_CLK_INIT(WIFI_SPI_CLK, ENABLE);

	/*!< Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(WIFI_SPI_SCK_GPIO_CLK | WIFI_SPI_MISO_GPIO_CLK |
						 WIFI_SPI_MOSI_GPIO_CLK | WIFI_CS_GPIO_CLK, ENABLE);

	/*!< SPI pins configuration *************************************************/
	// enable peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/*!< Connect SPI pins to AF5 */
	GPIO_PinAFConfig(WIFI_SPI_SCK_GPIO_PORT, WIFI_SPI_SCK_SOURCE, WIFI_SPI_SCK_AF);
	GPIO_PinAFConfig(WIFI_SPI_MISO_GPIO_PORT, WIFI_SPI_MISO_SOURCE, WIFI_SPI_MISO_AF);
	GPIO_PinAFConfig(WIFI_SPI_MOSI_GPIO_PORT, WIFI_SPI_MOSI_SOURCE, WIFI_SPI_MOSI_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	/*!< SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = WIFI_SPI_SCK_PIN;
	GPIO_Init(WIFI_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  WIFI_SPI_MOSI_PIN;
	GPIO_Init(WIFI_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  WIFI_SPI_MISO_PIN;
	GPIO_Init(WIFI_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

	/*!< Configure WIFI Card CS pin in output pushpull mode ********************/
	GPIO_InitStructure.GPIO_Pin = WIFI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(WIFI_CS_GPIO_PORT, &GPIO_InitStructure);

	SPI_InitTypeDef  SPI_InitStructure;


	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//configuration for cc3000 wifi
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(WIFI_SPI, &SPI_InitStructure);

	SPI_CalculateCRC(WIFI_SPI, DISABLE);

	SPI_Cmd(SPI2, ENABLE); // enable SPI2


	// Configure wifi_pwr_en on PD9
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_9);

	process_start(&wifi_spi_process, NULL);

}
/* The SPIWrite function transmits a given user buffer over the SPI.          */
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
	 unsigned char ucPad = 0;

	//
	// Figure out the total length of the packet in order to figure out if there is padding or not
	//
	if(!(usLength & 0x0001))
	{
		ucPad++;
	}

	fWlanInterruptDisable();
	WIFI_CS_LOW();
	if (pUserBuffer[6]==0&&pUserBuffer[7]==0x40)
		Delay(1);
	pUserBuffer[0]=0x01;
	pUserBuffer[1]=((usLength+ucPad) & 0xff00) >> 8;
	pUserBuffer[2]=(usLength+ucPad) & 0xff;
	pUserBuffer[3]=0;
	pUserBuffer[4]=0;

	long count=0;
	for (int i = 0; i < 4; i++)
	{
		SpiSendByte((uint8_t)pUserBuffer[i]);
		count++;
	}
	if (pUserBuffer[6]==0&&pUserBuffer[7]==0x40)
		Delay(1);
	for (int i = 4; i< usLength+5; i++)
	{
		SpiSendByte((uint8_t)pUserBuffer[i]);
		count++;
	}
	if (ucPad >0){
		SpiSendByte(0);
		count++;
	}

	WIFI_CS_HIGH();
    EXTI_ClearITPendingBit(EXTI_Line8);
	fWlanInterruptEnable();
	return count;
}

/* The SPIWrite function transmits a given user buffer over the SPI.          */
long SpiReceive(unsigned char *pUserBuffer)
{
	//disable interrrupt
	fWlanInterruptDisable();
	WIFI_CS_LOW();
	//Delay(1);
	long count=0;
	unsigned char lsb_size;
	int size;
	SpiSendByte(0x03);
	SpiSendByte(0);
	SpiSendByte(0);
	size=SpiSendByte(0);
	lsb_size=SpiSendByte(0);
	size = (size << 8) + lsb_size;

	for (int i = 0; i < size; i++)
	{
		pUserBuffer[i]= SpiSendByte(0);
		count++;
	}
	WIFI_CS_HIGH();

    EXTI_ClearITPendingBit(EXTI_Line8);
	fWlanInterruptEnable();

	return count;
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t SpiSendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(WIFI_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(WIFI_SPI, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(WIFI_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(WIFI_SPI);
}

/* The SpiRead function is called as a result of activity on the IRQ line     */
/* while the SPI is in IDLE state.                                            */
void SPI_IRQ(void)
{
	wlan_rx_buffer[0] = SpiSendByte(0);
	(*_pfRxHandler)();
}
/* The SPIClose function is called when the MCU decides to perform a shutdown */
/* operation on the CC3000 device. The functionality of SPIClose is up to the */
/* MCU. A general guideline is that it can shut down the SPI power domain and */
/* release the resources used by the SPI driver.                              */
void SpiClose(void)
{

}
/* The SpiResumeSpi function is called after the received packet is processed */
/* by the CC3000 host driver code in the context of the receive handler.      */
void  SpiResumeSpi(void)
{

}
//SpiWrite(pucBuff, ucArgsLength + SIMPLE_LINK_HCI_CMND_HEADER_SIZE);



void fWlanCB(long event_type, char * data, unsigned char length )
{

}

/* Callback provided during wlan_init call and invoked to read a value of     */
/* the SPI IRQ pin of the CC3000 device                                       */
long fWlanReadInteruptPin(void)
{
	//ITStatus itstatus = EXTI_GetITStatus(EXTI_Line0);
	//return itstatus;
	uint8_t status =  GPIO_ReadInputDataBit( GPIOD, GPIO_Pin_8);
	return status;
}

/* Callback provided during the wlan_init call and invoked to enable an       */
/* interrupt on the IRQ line of SPI                                           */
void fWlanInterruptEnable(void)
{
	EXTI->IMR |= EXTI_IMR_MR8;

}

/* Callback provided during the wlan_init call and invoked to disable an      */
/* interrupt on the IRQ line of SPI                                           */
void fWlanInterruptDisable(void)
{
	EXTI->IMR &= ~EXTI_IMR_MR8;

}

/* Callback provided during the wlan_init call and invoked to write a value   */
/* to the CC3000_EN pin of the CC3000 device, that is, entry or exit from     */
/* reset of the CC3000 device                                                 */
void fWriteWlanPin(unsigned char val)
{
	if(val){
		GPIO_SetBits(WIFI_CS_GPIO_PORT, WIFI_CS_PIN);
	}
	else{
		GPIO_ResetBits(WIFI_CS_GPIO_PORT, WIFI_CS_PIN);
	}
}

/**
  * @brief  Configures EXTI Line0 (connected to PD8 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTILine0_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource8);

  /* Configure EXTI WiFi */
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wifi_spi_process, ev, data)
{
  int len =0;

  PROCESS_BEGIN();

  while(1){
	  PROCESS_WAIT_EVENT();
	  len = SpiReceive(wlan_rx_buffer);
      SpiReceiveHandler(wlan_rx_buffer);

  }
//  printf("Hello, world\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    /* Set LED */
	  GPIO_SetBits(GPIOE, GPIO_Pin_12);
	  process_poll(&wifi_spi_process);
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line8);
  }
}
