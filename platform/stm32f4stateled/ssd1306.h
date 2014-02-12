/*
 * ssd1306.h
 *
 *  Created on: 11 Feb 2014
 *      Author: Jorge
 */

#ifndef SSD1306_H_
#define SSD1306_H_

//#include "cmsis_boot/stm32f10x.h"
#include "stm32f4xx_conf.h"

// Pin Definitions
//scl -> pa8
//sda -> pc9
//rst -> pc7
//d/c -> pc8
#define SSD1306_PORT                  GPIOC
//#define SSD1306_SCLK_PIN                   GPIO_Pin_0
//#define SSD1306_SDAT_PIN                   GPIO_Pin_1
#define SSD1306_RST_PIN                    GPIO_Pin_7
#define SSD1306_DC_PIN                     GPIO_Pin_8
//#define SSD1306_CS_PIN                     GPIO_Pin_4


#define SSD1306_LCDWIDTH                  128
#define SSD1306_LCDHEIGHT                 64

// Commands
#define SSD1306_SETCONTRAST               0x81
#define SSD1306_DISPLAYALLON_RESUME       0xA4
#define SSD1306_DISPLAYALLON              0xA5
#define SSD1306_NORMALDISPLAY             0xA6
#define SSD1306_INVERTDISPLAY             0xA7
#define SSD1306_DISPLAYOFF                0xAE
#define SSD1306_DISPLAYON                 0xAF
#define SSD1306_SETDISPLAYOFFSET          0xD3
#define SSD1306_SETCOMPINS                0xDA
#define SSD1306_SETVCOMDETECT             0xDB
#define SSD1306_SETDISPLAYCLOCKDIV        0xD5
#define SSD1306_SETPRECHARGE              0xD9
#define SSD1306_SETMULTIPLEX              0xA8
#define SSD1306_SETLOWCOLUMN              0x00
#define SSD1306_SETHIGHCOLUMN             0x10
#define SSD1306_SETSTARTLINE              0x40
#define SSD1306_MEMORYMODE                0x20
#define SSD1306_COMSCANINC                0xC0
#define SSD1306_COMSCANDEC                0xC8
#define SSD1306_SEGREMAP                  0xA0
#define SSD1306_CHARGEPUMP                0x8D
#define SSD1306_EXTERNALVCC               0x1
#define SSD1306_SWITCHCAPVCC              0x2






// Initialisation/Config Prototypes

void    ssd1306Init ( void );
void    ssd1306DrawPixel (uint8_t x, uint8_t y);
void    ssd1306ClearPixel (uint8_t x, uint8_t y);
//uint8_t ssd1306GetPixel ( uint8_t x, uint8_t y );
//void    ssd1306ClearScreen ( void );
void    ssd1306Refresh ( void );
//void    ssd1306DrawString( uint16_t x, uint16_t y, char* text, struct FONT_DEF font );
//void    ssd1306ShiftFrameBuffer( uint8_t height );


#endif /* SSD1306_H_ */
