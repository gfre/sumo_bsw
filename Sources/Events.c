/* ###################################################################
**     Filename    : Events.c
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
** @file Events.c
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         
/* MODULE Events */

#include "Cpu.h"
#include "Events.h"

#ifdef __cplusplus
extern "C" {
#endif 


/* User includes (#include below this line is not maintained by Processor Expert) */
#include "Tacho.h"
#include "rte.h"
#include "appl_cfg.h"
#include "portmacro.h"



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
void Cpu_OnNMIINT(void)
{
  /* Write your code here ... */
}

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
void Cpu_OnLossOfLockINT(void)
{
  /* Write your code here ... */
}

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
void FRTOS1_vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
  /* This will get called if a stack overflow is detected during the context
     switch.  Set configCHECK_FOR_STACK_OVERFLOWS to 2 to also check for stack
     problems within nested interrupts, but only do this for debug purposes as
     it will increase the context switch time. */
  (void)pxTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  /* Write your code here ... */
  for(;;) {}
}

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
void FRTOS1_vApplicationTickHook(void)
{
  /* Called for every RTOS tick. */
  /* Write your code here ... */
  TACHO_Sample();
  TRG1_AddTick();
}

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
void FRTOS1_vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
     free memory available in the FreeRTOS heap.  pvPortMalloc() is called
     internally by FreeRTOS API functions that create tasks, queues, software
     timers, and semaphores.  The size of the FreeRTOS heap is set by the
     configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  taskDISABLE_INTERRUPTS();
  /* Write your code here ... */
  for(;;) {}
}

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
void PTRC1_OnTraceWrap(void)
{
#if 0 /* default implementation for gdb below ... */
  /* Write your code here ... */
  uint8_t buf[64];

  /* GDB: dump binary memory <file> <hexStartAddr> <hexEndAddr> */
  PTRC1_vGetGDBDumpCommand(buf, sizeof(buf), "c:\\tmp\\trc.dump");
#endif
}

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
void KEY1_OnKeyPressed( uint8_t keys )
{
  /* Write your code here. A bit in 'keys' indicates key pressed ... */
  EvntCbFct_t *cbFct = NULL;

  cbFct = RTE_Get_BtnOnPrsdCbFct();
  if( NULL != cbFct )
  {
      cbFct( keys );
  }
}


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
void KEY1_OnKeyReleased(uint8_t keys)
{
  /* Write your code here. A bit in 'keys' indicates key released ... */
  EvntCbFct_t *cbFct = NULL;
  const APPL_TaskCfgItm_t *mainTaskCfg = NULL;
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  cbFct = RTE_Get_BtnOnRlsdCbFct();
  if(NULL != cbFct)
  {
      cbFct(keys);
  }
  mainTaskCfg = Get_APPL_MainTaskCfg();
  if( (NULL != mainTaskCfg ) && ( mainTaskCfg->taskHdl ) )
  {
      FRTOS1_xTaskNotifyFromISR( mainTaskCfg->taskHdl,
				 KEY_RELEASED_NOTIFICATION_VALUE,
				 eSetBits,
				 &higherPriorityTaskWoken );
      portYIELD_FROM_ISR( higherPriorityTaskWoken );
  }
}

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
void KEY1_OnKeyPressedLong(uint8_t keys)
{
  /* Write your code here ... */
  EvntCbFct_t *cbFct = NULL;
  const APPL_TaskCfgItm_t *mainTaskCfg = NULL;
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  cbFct = RTE_Get_BtnOnLngPrsdCbFct();
  if(NULL != cbFct)
  {
      cbFct(keys);
  }

  mainTaskCfg = Get_APPL_MainTaskCfg();
  if ((NULL != mainTaskCfg) && (mainTaskCfg->taskHdl))
  {
      FRTOS1_xTaskNotifyFromISR( mainTaskCfg->taskHdl,
				 KEY_PRESSED_LONG_NOTIFICATION_VALUE,
				 eSetBits,
				 &higherPriorityTaskWoken );
      portYIELD_FROM_ISR( higherPriorityTaskWoken );
  }
}

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
void QuadInt_OnInterrupt(void)
{
  Q4CLeft_Sample();
  Q4CRight_Sample();
  /* Write your code here ... */
}

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
void KEY1_OnKeyReleasedLong(uint8_t keys)
{
  /* Write your code here. A bit in 'keys' indicates key released after a long time ... */
  EvntCbFct_t *cbFct = NULL;

  cbFct = RTE_Get_BtnOnLngRlsdCbFct();
  if(NULL != cbFct)
  {
      cbFct(keys);
  }
}

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
void RNET1_OnRadioEvent(RNET1_RadioEvent event)
{
  /* Write your code here ... */
}

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

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
