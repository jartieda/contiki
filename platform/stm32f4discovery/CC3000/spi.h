#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define SPI_BUFFER_SIZE         1700
#define WIFI_SPI SPI2

#define WIFI_SPI_CLK                       RCC_APB1Periph_SPI2
#define WIFI_SPI_CLK_INIT                  RCC_APB1PeriphClockCmd

#define WIFI_SPI_SCK_PIN                   GPIO_Pin_13
#define WIFI_SPI_SCK_GPIO_PORT             GPIOB
#define WIFI_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define WIFI_SPI_SCK_SOURCE                GPIO_PinSource13
#define WIFI_SPI_SCK_AF                    GPIO_AF_SPI2

#define WIFI_SPI_MISO_PIN                  GPIO_Pin_14
#define WIFI_SPI_MISO_GPIO_PORT            GPIOB
#define WIFI_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define WIFI_SPI_MISO_SOURCE               GPIO_PinSource14
#define WIFI_SPI_MISO_AF                   GPIO_AF_SPI2

#define WIFI_SPI_MOSI_PIN                  GPIO_Pin_15
#define WIFI_SPI_MOSI_GPIO_PORT            GPIOB
#define WIFI_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define WIFI_SPI_MOSI_SOURCE               GPIO_PinSource15
#define WIFI_SPI_MOSI_AF                   GPIO_AF_SPI2

#define WIFI_CS_PIN                        GPIO_Pin_12
#define WIFI_CS_GPIO_PORT                  GPIOB
#define WIFI_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOB

#define WIFI_SPI_DMA                       DMA1
#define WIFI_SPI_DMA_CLK                   RCC_AHB1Periph_DMA1
#define WIFI_SPI_TX_DMA_CHANNEL            DMA_Channel_0
#define WIFI_SPI_TX_DMA_STREAM             DMA1_Stream4
#define WIFI_SPI_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF4
#define WIFI_SPI_RX_DMA_CHANNEL            DMA_Channel_0
#define WIFI_SPI_RX_DMA_STREAM             DMA1_Stream3
#define WIFI_SPI_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3

/* Exported macro ------------------------------------------------------------*/
/* Select WIFI: Chip Select pin low */
#define WIFI_CS_LOW()       GPIO_ResetBits(WIFI_CS_GPIO_PORT, WIFI_CS_PIN)
/* Deselect WIFI: Chip Select pin high */
#define WIFI_CS_HIGH()      GPIO_SetBits(WIFI_CS_GPIO_PORT, WIFI_CS_PIN)



extern uint8_t wlan_tx_buffer[SPI_BUFFER_SIZE];
extern uint8_t wlan_rx_buffer[SPI_BUFFER_SIZE];

/* The SpiOpen function is called from the wlan_start API function. The main  */
/* purpose of the function is to register the callback for the RX path,       */
/* allocate buffers for the SPI Transmission and Reception of data, and       */
/* initialize the SPI internal state-machine.                                 */
void SpiOpen(void (*pfRxHandler) );
/* The SPIWrite function transmits a given user buffer over the SPI.          */
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength);
uint8_t SpiSendByte(uint8_t byte);
/* The SpiRead function is called as a result of activity on the IRQ line     */
/* while the SPI is in IDLE state.                                            */
void SPI_IRQ(void);
/* The SPIClose function is called when the MCU decides to perform a shutdown */
/* operation on the CC3000 device. The functionality of SPIClose is up to the */
/* MCU. A general guideline is that it can shut down the SPI power domain and */
/* release the resources used by the SPI driver.                              */
void SpiClose(void);
/* The SpiResumeSpi function is called after the received packet is processed */
/* by the CC3000 host driver code in the context of the receive handler.      */
void  SpiResumeSpi(void);
/*                                                                            */
//SpiWrite(pucBuff, ucArgsLength + SIMPLE_LINK_HCI_CMND_HEADER_SIZE);

/*                                                                           */
void fWlanCB(long event_type, char * data, unsigned char length );
/* Callback provided during wlan_init call and invoked to read a value of     */
/* the SPI IRQ pin of the CC3000 device                                       */
long fWlanReadInteruptPin(void);
/* Callback provided during the wlan_init call and invoked to enable an       */
/* interrupt on the IRQ line of SPI                                           */
void fWlanInterruptEnable(void);
/* Callback provided during the wlan_init call and invoked to disable an      */
/* interrupt on the IRQ line of SPI                                           */
void fWlanInterruptDisable(void);
/* Callback provided during the wlan_init call and invoked to write a value   */
/* to the CC3000_EN pin of the CC3000 device, that is, entry or exit from     */
/* reset of the CC3000 device                                                 */
void fWriteWlanPin(unsigned char val);

/* Configures EXTI Line0 (connected to PD8 pin) in interrupt mode             */
void EXTILine0_Config(void);
