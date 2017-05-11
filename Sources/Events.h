/* ###################################################################
 **     Filename    : Events.h
 **     Project     : CAU_Zumo
 **     Processor   : MK22FX512VLK12
 **     Component   : Events
 **     Version     : Driver 01.00
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2017-01-03, 11:22, # CodeGen: 0
 **     Abstract    :
 **         This is user's event module.
 **         Put your event handler code here.
 **     Contents    :
 **         Cpu_OnNMIINT - void Cpu_OnNMIINT(void);
 **
 ** ###################################################################*/
/*!
 ** @file Events.h
 ** @version 01.00
 ** @brief
 **         This is user's event module.
 **         Put your event handler code here.
 */
/*!
 **  @addtogroup Events_module Events module documentation
 **  @{
 */

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
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
#include "FMSTR1.h"
#include "UART0.h"
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

#ifdef __cplusplus
extern "C" {
#endif 

/*
 ** ===================================================================
 **     Event       :  Cpu_OnNMIINT (module Events)
 **
 **     Component   :  Cpu [MK22FN1M0LQ12]
 */
/*!
 **     @brief
 **         This event is called when the Non maskable interrupt had
 **         occurred. This event is automatically enabled when the [NMI
 **         interrupt] property is set to 'Enabled'.
 */
/* ===================================================================*/
void Cpu_OnNMIINT(void);


/*
 ** ===================================================================
 **     Event       :  Cpu_OnLossOfLockINT (module Events)
 **
 **     Component   :  Cpu [MK22FN1M0LK12]
 */
/*!
 **     @brief
 **         This event is called when Loss of Lock interrupt had occured.
 **         This event is automatically enabled when either [Loss of
 **         lock interrupt] is set to 'Enabled'.
 */
/* ===================================================================*/
void Cpu_OnLossOfLockINT(void);

void FRTOS1_vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
/*
 ** ===================================================================
 **     Event       :  FRTOS1_vApplicationStackOverflowHook (module Events)
 **
 **     Component   :  FRTOS1 [FreeRTOS]
 **     Description :
 **         if enabled, this hook will be called in case of a stack
 **         overflow.
 **     Parameters  :
 **         NAME            - DESCRIPTION
 **         pxTask          - Task handle
 **       * pcTaskName      - Pointer to task name
 **     Returns     : Nothing
 ** ===================================================================
 */

void FRTOS1_vApplicationTickHook(void);
/*
 ** ===================================================================
 **     Event       :  FRTOS1_vApplicationTickHook (module Events)
 **
 **     Component   :  FRTOS1 [FreeRTOS]
 **     Description :
 **         If enabled, this hook will be called by the RTOS for every
 **         tick increment.
 **     Parameters  : None
 **     Returns     : Nothing
 ** ===================================================================
 */

void FRTOS1_vApplicationMallocFailedHook(void);
/*
 ** ===================================================================
 **     Event       :  FRTOS1_vApplicationMallocFailedHook (module Events)
 **
 **     Component   :  FRTOS1 [FreeRTOS]
 **     Description :
 **         If enabled, the RTOS will call this hook in case memory
 **         allocation failed.
 **     Parameters  : None
 **     Returns     : Nothing
 ** ===================================================================
 */

void PTRC1_OnTraceWrap(void);
/*
 ** ===================================================================
 **     Event       :  PTRC1_OnTraceWrap (module Events)
 **
 **     Component   :  PTRC1 [PercepioTrace]
 **     Description :
 **         Called for trace ring buffer wrap around. This gives the
 **         application a chance to dump the trace buffer.
 **     Parameters  : None
 **     Returns     : Nothing
 ** ===================================================================
 */

void KEY1_OnKeyPressed(uint8_t keys);
/*
 ** ===================================================================
 **     Event       :  KEY1_OnKeyPressed (module Events)
 **
 **     Component   :  KEY1 [Key]
 **     Description :
 **         Event generated at the time a key has been pressed.
 **     Parameters  :
 **         NAME            - DESCRIPTION
 **         keys            - the key(s) pressed, as bitset (e.g. 1 is
 **                           key 1, 2 is key 2, 4 is key 3, ....)
 **     Returns     : Nothing
 ** ===================================================================
 */

void KEY1_OnKeyReleased(uint8_t keys);
/*
 ** ===================================================================
 **     Event       :  KEY1_OnKeyReleased (module Events)
 **
 **     Component   :  KEY1 [Key]
 **     Description :
 **         Event generated after a key has been released.
 **     Parameters  :
 **         NAME            - DESCRIPTION
 **         keys            - the key(s) pressed, as bitset (e.g. 1 is
 **                           key 1, 2 is key 2, 4 is key 3, ....)
 **     Returns     : Nothing
 ** ===================================================================
 */

void KEY1_OnKeyPressedLong(uint8_t keys);
/*
 ** ===================================================================
 **     Event       :  KEY1_OnKeyPressedLong (module Events)
 **
 **     Component   :  KEY1 [Key]
 **     Description :
 **         Event generated at the time a long key press has been
 **         detected.
 **     Parameters  :
 **         NAME            - DESCRIPTION
 **         keys            - the key(s) pressed, as bitset (e.g. 1 is
 **                           key 1, 2 is key 2, 4 is key 3, ....)
 **     Returns     : Nothing
 ** ===================================================================
 */

/*
 ** ===================================================================
 **     Event       :  QuadInt_OnInterrupt (module Events)
 **
 **     Component   :  QuadInt [TimerInt]
 **     Description :
 **         When a timer interrupt occurs this event is called (only
 **         when the component is enabled - <Enable> and the events are
 **         enabled - <EnableEvent>). This event is enabled only if a
 **         <interrupt service/event> is enabled.
 **     Parameters  : None
 **     Returns     : Nothing
 ** ===================================================================
 */
void QuadInt_OnInterrupt(void);

void KEY1_OnKeyReleasedLong(uint8_t keys);
/*
** ===================================================================
**     Event       :  KEY1_OnKeyReleasedLong (module Events)
**
**     Component   :  KEY1 [Key]
**     Description :
**         Event generated after a key has been released (long key
**         press).
**     Parameters  :
**         NAME            - DESCRIPTION
**         keys            - the key(s) pressed, as bitset (e.g. 1 is
**                           key 1, 2 is key 2, 4 is key 3, ....)
**     Returns     : Nothing
** ===================================================================
*/

void RNET1_OnRadioEvent(RNET1_RadioEvent event);
/*
** ===================================================================
**     Event       :  RNET1_OnRadioEvent (module Events)
**
**     Component   :  RNET1 [RNet]
**     Description :
**         Event created for various radio states, like timeout, ack
**         received, data sent, ...
**     Parameters  :
**         NAME            - DESCRIPTION
**         event           - 
**     Returns     : Nothing
** ===================================================================
*/

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
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
