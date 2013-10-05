#include "spi.h"
#include <stdint.h>
#include <stm32f4xx_spi.h>
#include "stm32f4xx_conf.h"
#include <contiki.h>

/*---------------------------------------------------------------------------*/

#define SPI_BUFFER_SIZE         1700


uint8_t wlan_tx_buffer[SPI_BUFFER_SIZE];
uint8_t wlan_rx_buffer[SPI_BUFFER_SIZE];
int ubRxIndex=0;
int ubTxIndex=0;
int ubRxMax=0;
int ubTxMax=0;
uint8_t wifi_state=3;//0 tx, 1 rx header, 2 rx body , 3 tx Init;


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

//	//Initialize DMA
//	DMA_InitTypeDef DMA_InitStructure;
//	/* Enable DMA clock */
//	RCC_AHB1PeriphClockCmd(WIFI_SPI_DMA_CLK, ENABLE);
//
//	/* DMA configuration -------------------------------------------------------*/
//	/* Deinitialize DMA Streams */
//	DMA_DeInit(WIFI_SPI_TX_DMA_STREAM);
//	DMA_DeInit(WIFI_SPI_RX_DMA_STREAM);
//
//	/* Configure DMA Initialization Structure */
//	DMA_InitStructure.DMA_BufferSize = SPI_BUFFER_SIZE;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(WIFI_SPI->DR));
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	/* Configure TX DMA */
//	DMA_InitStructure.DMA_Channel = WIFI_SPI_TX_DMA_CHANNEL;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)wlan_tx_buffer;
//	DMA_Init(WIFI_SPI_TX_DMA_STREAM, &DMA_InitStructure);
//	/* Configure RX DMA */
//	DMA_InitStructure.DMA_Channel = WIFI_SPI_RX_DMA_CHANNEL;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)wlan_rx_buffer;
//	DMA_Init(WIFI_SPI_RX_DMA_STREAM, &DMA_InitStructure);
//
//	/* Enable DMA SPI TX Stream */
//	DMA_Cmd(WIFI_SPI_TX_DMA_STREAM,ENABLE);
//
//	/* Enable DMA SPI RX Stream */
//	DMA_Cmd(WIFI_SPI_RX_DMA_STREAM,ENABLE);
//
//	/* Enable SPI DMA TX Requsts */
//	SPI_I2S_DMACmd(WIFI_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
//
//	/* Enable SPI DMA RX Requsts */
//	SPI_I2S_DMACmd(WIFI_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

	/* Enable the SPI2 */
	SPI_Cmd(SPI2, ENABLE); // enable SPI2

	/* Configure the Priority Group to 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the SPI interrupt priority */
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Configure wifi_pwr_en on PD9
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_9);


	/* Enable the Rx buffer not empty interrupt */
	SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_RXNE, ENABLE);

	/* Enable the Tx buffer empty interrupt */
	SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_TXE, ENABLE);

}
/* The SPIWrite function transmits a given user buffer over the SPI.          */
long SpiWrite_Init(unsigned char *pUserBuffer, unsigned short usLength)
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
	//while(!WIFI_CS_CHECK()){};
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
//	while (SPI_I2S_GetFlagStatus(WIFI_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	Delay(10);
	WIFI_CS_HIGH();
	EXTI_ClearITPendingBit(EXTI_Line8);

	//enable EXIT_IRQ from CC3000
	fWlanInterruptEnable();

	return count;
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

	while(!(WIFI_CS_CHECK() && fWlanReadInteruptPin()))
	{
		Delay(1);
	};
	fWlanInterruptDisable();
	WIFI_CS_LOW();
	pUserBuffer[0]=0x01;
	pUserBuffer[1]=((usLength+ucPad) & 0xff00) >> 8;
	pUserBuffer[2]=(usLength+ucPad) & 0xff;
	pUserBuffer[3]=0;
	pUserBuffer[4]=0;
	if (ucPad >0){
		pUserBuffer[usLength+5]=0;
	}
	ubTxIndex = 1;//index to 1 because I send first byte by hand
	ubTxMax = usLength+ucPad+5;
	ubRxIndex = 0;//index to 1 because I send first byte by hand
	ubRxMax = usLength+ucPad+5;
	wifi_state=0;
	SpiSendByte((uint8_t)pUserBuffer[0]);//first byte send int send the rest
	SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_TXE, ENABLE);

	return 0;//FIXME RETURN COUNT?
}

/* The SPIWrite function transmits a given user buffer over the SPI.          */
long SpiReceive(unsigned char *pUserBuffer)
{
	//disable interrrupt
	fWlanInterruptDisable();

	WIFI_CS_LOW();
	//Delay(1);
	wlan_tx_buffer[0]=0x03;
	wlan_tx_buffer[1]=0x0;
	wlan_tx_buffer[2]=0x0;
	wlan_tx_buffer[3]=0x0;
	wlan_tx_buffer[4]=0x0;
	ubRxMax=5;
	ubTxMax=5;
	ubTxIndex = 1;
	ubRxIndex = 0;
	wifi_state=1;//rx header
	SpiSendByte(wlan_tx_buffer[0]);//send first next are sent on int
	SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_TXE, ENABLE);

//	WIFI_CS_HIGH();

//    EXTI_ClearITPendingBit(EXTI_Line8);
//	fWlanInterruptEnable();

	return 0;//FIXME THIS RETURNED COUNT
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
  //while (SPI_I2S_GetFlagStatus(WIFI_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  //return SPI_I2S_ReceiveData(WIFI_SPI);
  return 0;
}

/* The SpiRead function is called as a result of activity on the IRQ line     */
/* while the SPI is in IDLE state.                                            */
void SPI2_IRQHandler(void)
{

  /* SPI in Receiver mode */
  if (SPI_I2S_GetITStatus(WIFI_SPI, SPI_I2S_IT_RXNE) == SET)
  {

	if (ubRxIndex < ubRxMax && ubRxIndex < SPI_BUFFER_SIZE)
	{
	  /* Receive Transaction data */
		wlan_rx_buffer[ubRxIndex++] = SPI_I2S_ReceiveData(WIFI_SPI);
		if (wifi_state == 1 )//rx header
		{
			if(ubRxIndex == 5){
				unsigned char lsb_size;
				int size;
				size=wlan_rx_buffer[3];
				lsb_size=wlan_rx_buffer[4];
				size = (size << 8) + lsb_size;

				for (int i = 0; i < size; i++)
				{
					wlan_tx_buffer[i]= 0;
				}
				wifi_state=2; //rx body
				ubTxMax = size;
				ubTxIndex = 1 ;
				ubRxMax = size + 5;
				SpiSendByte(wlan_tx_buffer[0]); //sends first byte so next are sent by interrupt
				SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_TXE, ENABLE);

			}
		}else if (wifi_state==2)//rx body
		{
			if (ubRxIndex == ubRxMax){ //end reception
				WIFI_CS_HIGH();

				EXTI_ClearITPendingBit(EXTI_Line8);
				fWlanInterruptEnable();
				SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_TXE, DISABLE);

				(*_pfRxHandler)(wlan_rx_buffer+5);
			}
		}else if (ubTxIndex == ubTxMax &&ubRxIndex == ubRxMax && wifi_state == 0){//end of transsmision

			WIFI_CS_HIGH();
		    EXTI_ClearITPendingBit(EXTI_Line8);
			fWlanInterruptEnable();
			SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_TXE, DISABLE);

		}
	}
	else
	{
	  /* Disable the Rx buffer not empty interrupt */
		SPI_I2S_ReceiveData(WIFI_SPI);
	  //SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_RXNE, DISABLE);
	}
  }
  /* SPI in Tramitter mode */
  if (SPI_I2S_GetITStatus(WIFI_SPI, SPI_I2S_IT_TXE) == SET)
  {
	if ((ubTxIndex < ubTxMax && ubTxIndex < SPI_BUFFER_SIZE))//||wifi_state==1|| wifi_state==2)
	{
	  /* Send Transaction data */
	  SPI_I2S_SendData(WIFI_SPI, wlan_tx_buffer[ubTxIndex++]);
	}
	else
	{
	  /* Disable the Tx buffer empty interrupt */
	  SPI_I2S_ITConfig(WIFI_SPI, SPI_I2S_IT_TXE, DISABLE);
	}
  }
	//wlan_rx_buffer[0] = SpiSendByte(0);
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
void EXTILine_Config(void)
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
	  SpiReceive(wlan_rx_buffer);
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line8);
  }
}
