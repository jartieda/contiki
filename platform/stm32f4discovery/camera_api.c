/**
  ******************************************************************************
  * @file    DCMI/DCMI_CameraExample/camera_api.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-January-2013
  * @brief   This file contains the routinue needed to configure OV9655/OV2640 
  *          Camera modules.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "camera_api.h"
#include "dcmi_ov9655.h"
#include "dcmi_ov2640.h"
#include "stdint.h"
#include "stm32f4xx_conf.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup DCMI_CameraExample
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Image Formats */
const uint8_t *ImageForematArray[] =
{
  (uint8_t*)"BMP QQVGA Format    ",
  (uint8_t*)"BMP QVGA Format     ",
};
static __IO uint32_t TimingDelay;
__IO uint32_t PressedKey = 0;
uint8_t ValueMin = 0, ValueMax = 0;
Camera_TypeDef Camera;
ImageFormat_TypeDef ImageFormat;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize resources used for this demo. 
  * @param  None
  * @retval Digit value
  */
uint8_t Demo_Init(void)
{
  PressedKey = NOKEY;
  
  return PressedKey;
 /* Configure the IO Expander */
}

/**
  * @brief  Display the navigation menu.
  * @param  None
  * @retval None
  */
void Display_Menu(uint8_t ForematIndex, uint8_t MaxForematIndex)
{

}

/**
  * @brief  Configures OV9655 or OV2640 Camera module mounted on STM324xG-EVAL board.
  * @param  ImageBuffer: Pointer to the camera configuration structure
  * @retval None
  */
void Camera_Config(void)
{
  if(Camera == OV9655_CAMERA)
  {
//    switch (ImageFormat)
//    {
//      case BMP_QQVGA:
//      {
//        /* Configure the OV9655 camera and set the QQVGA mode */
//        OV9655_HW_Init();
//        OV9655_Init(BMP_QQVGA);
//        OV9655_QQVGAConfig();
//        break;
//      }
//      case BMP_QVGA:
//      {
//        /* Configure the OV9655 camera and set set the QVGA mode */
//        OV9655_HW_Init();
//        OV9655_Init(BMP_QVGA);
//        OV9655_QVGAConfig();
//        break;
//      }
//      default:
//      {
//        /* Configure the OV9655 camera and set the QQVGA mode */
//        OV9655_HW_Init();
//        OV9655_Init(BMP_QQVGA);
//        OV9655_QQVGAConfig();
//        break;
//      }
//    }
  }
  else if(Camera == OV2640_CAMERA)
  {
    switch (ImageFormat)
    {
      case BMP_QQVGA:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
        OV2640_HW_Init();
        OV2640_Init(BMP_QQVGA);
        OV2640_QQVGAConfig();
        break;
      }
      case BMP_QVGA:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
        OV2640_HW_Init();
        OV2640_Init(BMP_QVGA);
        OV2640_QVGAConfig();
        break;
      }
      default:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
        OV2640_HW_Init();
        OV2640_Init(BMP_QQVGA);
        OV2640_QQVGAConfig();
        break; 
      }
    }
  }
}

/**
  * @brief  OV2640 camera special effects.
* @param  index: 
  * @retval None
  */
void OV2640_SpecialEffects(uint8_t index)
{
  switch (index)
  {
    case 1:
    {
      //LCD_DisplayStringLine(LINE(16), (uint8_t*)" Antique               ");
      OV2640_ColorEffectsConfig(0x40, 0xa6);/* Antique */ 
      break;
    }
    case 2:
    {
      //LCD_DisplayStringLine(LINE(16), (uint8_t*)" Bluish                ");
      OV2640_ColorEffectsConfig(0xa0, 0x40);/* Bluish */
      break;
    }
    case 3:
    {
      //LCD_DisplayStringLine(LINE(16), (uint8_t*)" Greenish              ");
      OV2640_ColorEffectsConfig(0x40, 0x40);/* Greenish */
      break;
    }
    case 4:
    {
      //LCD_DisplayStringLine(LINE(16), (uint8_t*)" Reddish               ");
      OV2640_ColorEffectsConfig(0x40, 0xc0);/* Reddish */
      break;
    }
    case 5:
    {
      //LCD_DisplayStringLine(LINE(16), (uint8_t*)" Black & White         ");
      OV2640_BandWConfig(0x18);/* Black & White */
      break;
    }
    case 6:
    {
      //LCD_DisplayStringLine(LINE(16), (uint8_t*)" Negative              ");
      OV2640_BandWConfig(0x40);/* Negative */
      break;
    }
    case 7:
    {
      //LCD_DisplayStringLine(LINE(16), (uint8_t*)" Black & White negative");
      OV2640_BandWConfig(0x58);/* B&W negative */
      break;
    }
    case 8:
    {
      //LCD_DisplayStringLine(LINE(16), (uint8_t*)" Normal                ");
      OV2640_BandWConfig(0x00);/* Normal */
      break;
    }
    default:
      break;
  }
}



/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds
  * @retval None
  */
void Delay_(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  Empty function.
  * @param  None
  * @retval None
  */
void NullFunc(void)
{
}

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
