/* ###################################################################
**     Filename    : main.c
**     Project     : CAU_Zumo
**     Processor   : MK22FX512VLK12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2017-01-03, 11:22, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "FRTOS1.h"
#include "RTOSCNTRLDD1.h"
#include "SM1.h"
#include "SMasterLdd1.h"
#include "RF1.h"
#include "CE1.h"
#include "BitIoLdd11.h"
#include "CSN1.h"
#include "BitIoLdd12.h"
#include "IRQ1.h"
#include "ExtIntLdd3.h"
#include "RNET1.h"
#include "AD1.h"
#include "KIN1.h"
#include "USB1.h"
#include "CDC1.h"
#include "Tx1.h"
#include "Rx1.h"
#include "USB0.h"
#include "TMOUT1.h"
#include "IFsh1.h"
#include "IntFlashLdd1.h"
#include "PTC.h"
#include "BT1.h"
#include "Serial1.h"
#include "ASerialLdd1.h"
#include "FMSTR1.h"
#include "MCUC1.h"
#include "MOTTU.h"
#include "DIRL.h"
#include "BitIoLdd1.h"
#include "PWMR.h"
#include "PwmLdd1.h"
#include "DIRR.h"
#include "BitIoLdd2.h"
#include "PWML.h"
#include "PwmLdd2.h"
#include "Q4CLeft.h"
#include "C11.h"
#include "BitIoLdd3.h"
#include "C21.h"
#include "BitIoLdd4.h"
#include "Q4CRight.h"
#include "C12.h"
#include "BitIoLdd5.h"
#include "C23.h"
#include "BitIoLdd6.h"
#include "QuadInt.h"
#include "TimerIntLdd1.h"
#include "TU_QuadInt.h"
#include "LED1.h"
#include "LEDpin1.h"
#include "BitIoLdd7.h"
#include "LED2.h"
#include "LEDpin2.h"
#include "BitIoLdd8.h"
#include "BUZ1.h"
#include "BitIoLdd9.h"
#include "KEY1.h"
#include "keyPin1.h"
#include "KeyISRpin1.h"
#include "ExtIntLdd2.h"
#include "TRG1.h"
#include "PTA.h"
#include "UTIL1.h"
#include "CLS1.h"
#include "WAIT1.h"
#include "XF1.h"
#include "CS1.h"
#include "RTT1.h"

/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* User includes (#include below this line is not maintained by Processor Expert) */
#include "task.h"

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */
  TASK_Init();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
