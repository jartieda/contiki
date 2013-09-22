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
*			Watchdog
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*/

#include <stdio.h>
#include "dev/watchdog.h"
#include "stm32f4xx_conf.h"


/*---------------------------------------------------------------------------*/
void
watchdog_init(void)
{
  /* WWDG configuration */
  /* Enable WWDG clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

  /* WWDG clock counter = (PCLK1 (42MHz)/4096)/8 = 1281 Hz (~780 us)  */
  WWDG_SetPrescaler(WWDG_Prescaler_8);

  /* Set Window value to 80; WWDG counter should be refreshed only when the counter
    is below 80 (and greater than 64) otherwise a reset will be generated */
  WWDG_SetWindowValue(80);


}
/*---------------------------------------------------------------------------*/
void
watchdog_start(void)
{
    /* Enable WWDG and set counter value to 127, WWDG timeout = ~780 us * 64 = 49.92 ms 
     In this case the refresh window is: 
           ~780 * (127-80) = 36.6ms < refresh window < ~780 * 64 = 49.9ms
  */
  WWDG_Enable(127);

}
/*---------------------------------------------------------------------------*/
void
watchdog_periodic(void)
{
  /* This function is called periodically to restart the watchdog timer. */
    /* Update WWDG counter */
    WWDG_SetCounter(127);
}
/*---------------------------------------------------------------------------*/
void
watchdog_stop(void)
{

}
/*---------------------------------------------------------------------------*/
void
watchdog_reboot(void)
{

}
/*---------------------------------------------------------------------------*/
/** @} */
