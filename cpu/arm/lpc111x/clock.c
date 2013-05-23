
#include "LPC11xx.h"                        /* LPC11xx definitions */

#include "sys/clock.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"

volatile uint32_t msTicks;                            /* counts 1ms timeTicks */
volatile unsigned long seconds;

/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
  if((msTicks%1000) == 0)
  {
    seconds++;
  }
}

/*------------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *------------------------------------------------------------------------------*/
void clock_delay(unsigned int dlyTicks) {
	unsigned int curTicks;

	curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks);
}

/*---------------------------------------------------------------------------*/
void clock_init(void)
{
	if (SysTick_Config(SystemCoreClock / 1000)) { /* Setup SysTick Timer for 1 msec interrupts  */
		while (1);                                  /* Capture error */
	}

	if ( !(SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk) )
	{
		/* When external reference clock is used(CLKSOURCE in
		Systick Control and register bit 2 is set to 0), the
		SYSTICKCLKDIV must be a non-zero value and 2.5 times
		faster than the reference clock.
		When core clock, or system AHB clock, is used(CLKSOURCE
		in Systick Control and register bit 2 is set to 1), the
		SYSTICKCLKDIV has no effect to the SYSTICK frequency. See
		more on Systick clock and status register in Cortex-M3
		technical Reference Manual. */
		LPC_SYSCON->SYSTICKCLKDIV = 0x08;
	  }
}

/*---------------------------------------------------------------------------*/
clock_time_t clock_time(void)
{
  clock_time_t tmp;
  tmp = (clock_time_t)msTicks;

  return tmp;
}

/*---------------------------------------------------------------------------*/
void clock_adjust_seconds(uint8_t howmany) {
   seconds += howmany;
//   sleepseconds +=howmany;
}

/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 1 / 125 sec = 0.008 ms.
 *
 */
void clock_wait(int i)
{
  clock_time_t start;

  start = clock_time();
  while(clock_time() - start < (clock_time_t)i);
}

/*---------------------------------------------------------------------------*/
void clock_set_seconds(unsigned long sec)
{
    // TODO
}

unsigned long clock_seconds(void)
{
  unsigned long tmp;

  tmp = seconds;
  return tmp;
}

rtimer_clock_t clock_counter(void)
{
  return seconds;
}
