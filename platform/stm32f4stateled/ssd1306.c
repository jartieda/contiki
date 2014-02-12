/*
 * ssd1306.c
 *
 *  Created on: 11 Feb 2014
 *      Author: Jorge
 */

#include "ssd1306.h"
#include "stm32f4xx_conf.h"

#define I2C_TIMEOUT_MAX 10000

void ssd1306SendByte(uint8_t byte,uint8_t cmd_or_data);
void DELAY();
void CMD();
void DATA();
uint8_t SPI1_send(uint8_t data);

uint8_t buffer[SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8];

//-------------------------------- DRIVER INTERNAL FUNCTIONS --------------------------------------

void CMD(uint32_t c)
{
//   GPIO_WriteBit( SSD1306_PORT, SSD1306_CS_PIN, 1 );
   GPIO_WriteBit( SSD1306_PORT, SSD1306_DC_PIN, 0 );
//   GPIO_WriteBit( SSD1306_PORT, SSD1306_CS_PIN, 0 );
	//ssd1306SendByte(c,0);
	SPI1_send(c);
//   GPIO_WriteBit( SSD1306_PORT, SSD1306_CS_PIN, 1 );
}

void DATA(uint32_t d)
{
//   GPIO_WriteBit( SSD1306_PORT, SSD1306_CS_PIN, 1 );
   GPIO_WriteBit( SSD1306_PORT, SSD1306_DC_PIN, 1 );
//   GPIO_WriteBit( SSD1306_PORT, SSD1306_CS_PIN, 0 );
   //ssd1306SendByte( d , 1);
   SPI1_send(d);
//   GPIO_WriteBit( SSD1306_PORT, SSD1306_CS_PIN, 1 );
}



void DELAY(uint32_t value) // delay
{
	Delay(value);
/*   TIM2->ARR = value;
   TIM2->CNT = 0;
   TIM2->SR &= ~TIM_SR_UIF;
   TIM2->CR1 = TIM_CR1_CEN;
   while((TIM2->SR & TIM_SR_UIF)==0){}
   TIM2->SR &= ~TIM_SR_UIF;*/
}

void ssd1306SendByte(uint8_t byte, uint8_t cmd_or_data)
{
//	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);

	/*!< Send byte through the SPI1 peripheral */
//	SPI_I2S_SendData(SPI3, byte);

	SPI3->DR = byte; // write data to be transmitted to the SPI data register
	while( !(SPI3->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI3->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI3->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	//return SPI3->DR; // return received data from SPI data register


}
void ssd1306SendByte_i2c(uint8_t byte, uint8_t cmd_or_data)
{
	//write addres
	uint32_t timeout = I2C_TIMEOUT_MAX;
	/* Generate the Start Condition */
	I2C_GenerateSTART(I2C3, ENABLE);

	/* Test on I2C3 EV5, Start trnsmitted successfully and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT))
	{
	   /* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0) return ;
	}

	/* Send Memory device slave Address for write */
	I2C_Send7bitAddress(I2C3, 0b01111000, I2C_Direction_Transmitter);

	/* Test on I2C3 EV6 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0){
			//return ;
			break;
		}
	}

	/* Send I2C3 location address LSB */
	uint8_t control;
	if (!cmd_or_data){
		control = 0b00000000; //command
	}else{
		control = 0b01000000; //data
	}
	I2C_SendData(I2C3, control);

	/* Test on I2C3 EV8 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
		{
			//return ;
			break;
		}
	}

	/* Send I2C3 location address LSB */
	uint8_t data = 0xFF && byte; //databayte
	I2C_SendData(I2C3, data);

	/* Test on I2C3 EV8 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0) {
//			return;
			break;
		}
	}

}


//-------------------------------- OLED FUNCTIONS --------------------------------------

void init_i2c3(void)
{
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as I2C3SDA and I2C3SCL
	GPIO_InitTypeDef GPIO_Output;     // For some debugging LEDs
	I2C_InitTypeDef I2C_InitStruct; // this is for the I2C3 initilization


	/* enable the peripheral clock for the pins used by
	PB6 for I2C SCL and PB9 for I2C3_SDL*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


	/* enable APB1 peripheral clock for I2C3*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);

	/* This sequence sets up the I2C3SDA and I2C3SCL pins
	* so they work correctly with the I2C3 peripheral
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8; // Pins A8(I2C3_SCL) and C9(I2C3_SDA)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;// this defines the output type as open drain
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;// this activates the pullup resistors on the IO pins

	GPIO_Init(GPIOA, &GPIO_InitStruct);// now all the values are passed to the GPIO_Init()
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; // Pins A8(I2C3_SCL) and C9(I2C3_SDA)
	GPIO_Init(GPIOC, &GPIO_InitStruct);// now all the values are passed to the GPIO_Init()


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	/*GPIO_Output.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_Output.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Output.GPIO_OType = GPIO_OType_PP;
	GPIO_Output.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Output.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_Output);*/

	/* The I2C3_SCL and I2C3_SDA pins are now connected to their AF
	* so that the I2C3 can take over control of the
	* pins
	*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3); //
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);

	/* Configure I2C3 */
	I2C_DeInit(I2C3);

	/* Enable the I2C peripheral */
	I2C_Cmd(I2C3, ENABLE);

	/* Set the I2C structure parameters */
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0xEE;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 30000;

	/* Initialize the I2C peripheral w/ selected parameters */
	I2C_Init(I2C3,&I2C_InitStruct);

}
void spi2_init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/*!< Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	/*!< Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	/*!< SPI pins configuration *************************************************/
	// enable peripheral clock

	/*!< Connect SPI pins to AF5 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	/*!< SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*!< SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_PinSource11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*!< SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_PinSource12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*!< Configure WIFI Card CS pin in output pushpull mode ********************/
	//GPIO_InitStructure.GPIO_Pin = WIFI_CS_PIN;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_Init(WIFI_CS_GPIO_PORT, &GPIO_InitStructure);

	SPI_InitTypeDef  SPI_InitStructure;

	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//configuration for cc3000 wifi
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS =  SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI3, &SPI_InitStructure);

	SPI_CalculateCRC(SPI3, DISABLE);
	/* Enable the SPI2 */
	SPI_Cmd(SPI3, ENABLE); // enable SPI2

}
// this function initializes the SPI1 peripheral
void init_SPI1(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure the chip select pin
	   in this case we will use PE7 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIOE->BSRRL |= GPIO_Pin_7; // set PE7 high

	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* configure SPI1 in Mode 0
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE); // enable SPI1
}

/* This funtion is used to transmit and receive data
 * with SPI1
 * 			data --> data to be transmitted
 * 			returns received value
 */
uint8_t SPI1_send(uint8_t data){

	SPI1->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI1->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI1->DR; // return received data from SPI data register
}

void ssd1306Init(void)
{

	init_SPI1();
   DELAY(1000);
  // Set all pins to output
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef gpio;
  GPIO_StructInit(&gpio);
  gpio.GPIO_Mode = GPIO_Mode_OUT;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Pin =  SSD1306_DC_PIN | SSD1306_RST_PIN  ;
  gpio.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(SSD1306_PORT, &gpio);

  //delay init TIM2
  //RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  //TIM2->PSC     = 24000-1;
  //TIM2->CR1 = TIM_CR1_CEN;

  // Reset the LCD
  //GPIO_WriteBit(SSD1306_PORT, SSD1306_CS_PIN, 0);

  GPIO_WriteBit( SSD1306_PORT, SSD1306_DC_PIN, 0 );//DC->SA0->Low:

  GPIO_WriteBit(SSD1306_PORT, SSD1306_RST_PIN, 1);
  DELAY(1);
  GPIO_WriteBit(SSD1306_PORT, SSD1306_RST_PIN, 0);
  DELAY(10);
  GPIO_WriteBit(SSD1306_PORT, SSD1306_RST_PIN, 1);
  DELAY(10);



  // Initialisation sequence
  CMD(SSD1306_DISPLAYOFF);                    // 0xAE
  CMD(SSD1306_SETLOWCOLUMN | 0x0);            // low col = 0
  CMD(SSD1306_SETHIGHCOLUMN | 0x0);           // hi col = 0
  CMD(SSD1306_SETSTARTLINE | 0x0);            // line #0
  CMD(SSD1306_SETCONTRAST);                   // 0x81
  CMD(0xCF);                            // set internal power
  CMD(0xa1);                                  // setment remap 95 to 0 (?)
  CMD(SSD1306_NORMALDISPLAY);                 // 0xA6
  //CMD(SSD1306_INVERTDISPLAY);
  CMD(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  CMD(SSD1306_SETMULTIPLEX);                  // 0xA8
  CMD(0x3F);                                  // 0x3F 1/64 duty
  CMD(SSD1306_SETDISPLAYOFFSET);              // 0xD3
  CMD(0x0);                                   // no offset
  CMD(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
  CMD(0x80);                                  // the suggested ratio 0x80
  CMD(SSD1306_SETPRECHARGE);                  // 0xd9
  CMD(0xF1);                            // set internal power
  CMD(SSD1306_SETCOMPINS);                    // 0xDA
  CMD(0x12);                                  // disable COM left/right remap
  CMD(SSD1306_SETVCOMDETECT);                 // 0xDB
  CMD(0x40);                                  // 0x20 is default?
  CMD(SSD1306_MEMORYMODE);                    // 0x20
  CMD(0x00);                                  // 0x0 act like ks0108
  CMD(SSD1306_SEGREMAP | 0x1);
  CMD(SSD1306_COMSCANDEC);
  CMD(SSD1306_CHARGEPUMP);                    //0x8D
  CMD(0x14);                             // set internal power
  CMD(SSD1306_DISPLAYON);

  //GPIO_WriteBit(SSD1306_PORT, SSD1306_CS_PIN, 1);

}


void ssd1306Refresh(void)
{
  CMD(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
  CMD(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
  CMD(SSD1306_SETSTARTLINE | 0x0); // line #0

  uint16_t i;
  for (i=0; i<1024; i++)
  {
    DATA(buffer[i]);
  }
}



void ssd1306DrawPixel(uint8_t x, uint8_t y)
{
  if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
    return;

  buffer[x+ (y/8)*SSD1306_LCDWIDTH] |= (1 << y%8);
}

void ssd1306ClearPixel(uint8_t x, uint8_t y)
{
  if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
    return;

  buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~(1 << y%8);
}


