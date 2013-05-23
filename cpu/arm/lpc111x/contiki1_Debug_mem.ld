/*
 * GENERATED FILE - DO NOT EDIT
 * (C) Code Red Technologies Ltd, 2008-2013
 * Linker script memory definitions
 * Created from LinkMemoryTemplate
 * By LPCXpresso v5.1.0 [Build 2048] [2013-02-05]  on Thu May 23 15:45:44 CEST 2013)
*/

MEMORY
{
  /* Define each memory region */
  MFlash56 (rx) : ORIGIN = 0x0, LENGTH = 0xe000 /* 56k */
  RamLoc8 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x2000 /* 8k */

}
  /* Define a symbol for the top of each memory region */
  __top_MFlash56 = 0x0 + 0xe000;
  __top_RamLoc8 = 0x10000000 + 0x2000;

