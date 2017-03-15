/***************************************************************************************************
 * @brief 	This module creates and runs all tasks
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file task.c
 * 
 *==================================================================================================
 */

#define MASTER_task_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "task.h"
#include "task_cfg.h"
#include "task_Types.h"
#include "Motor.h"
#include "RNET1.h"
#include "Tacho.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "Pid.h"
#include "drv.h"
#include "rnet.h"
#include "appl.h"

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void TASK_AdoptToHardware(void);
static void TASK_CreateTasks();



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void TASK_AdoptToHardware(void)
{
	/*Motor direction & Quadrature configuration for CAU_ZUMO */
	(void)Q4CRight_SwapPins(TRUE);
	MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert left motor */
	MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert right motor */

	/* SW1: enable and turn on pull-up resistor for PTA14 (push button) */
	PORT_PDD_SetPinPullSelect(PORTA_BASE_PTR, 14, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTA_BASE_PTR, 14, PORT_PDD_PULL_ENABLE);

	/* pull-ups for Quadrature Encoder Pins */
	PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 10, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 10, PORT_PDD_PULL_ENABLE);
	PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 11, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 11, PORT_PDD_PULL_ENABLE);
	PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 16, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 16, PORT_PDD_PULL_ENABLE);
	PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 17, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 17, PORT_PDD_PULL_ENABLE);
}


static void TASK_CreateTasks()
{
	uint8 i = 0u;
	const TASK_Cfg_t *taskCfg = NULL;
	taskCfg = TASK_Get_TasksCfg();

	if(NULL != taskCfg)
	{
		for(i = 0u; i < taskCfg->numTasks; i++)
		{
			if(NULL != &(taskCfg->tasks[i]))
			{
				if(pdPASS != FRTOS1_xTaskCreate(taskCfg->tasks[i].taskFctHdl,
						taskCfg->tasks[i].taskName,
					    taskCfg->tasks[i].stackDepth,
					    taskCfg->tasks[i].pvParameters,
					    taskCfg->tasks[i].taskPriority,
					   &taskCfg->tasks[i].taskHdl))
				{
					/* The task could not be created because there was not enough
					 * FreeRTOS heap memory available for the task data structures and
					 * stack to be allocated.
					 */
				} /* !pdPASS */
				else
				{
					/* The task was created successfully */
					if(TASK_SUSP_DEFAULT == taskCfg->tasks[i].suspTask)
					{
						FRTOS1_vTaskSuspend(taskCfg->tasks[i].taskHdl);
					}
				} /* pdPASS */
			}
		}
		FRTOS1_vTaskStartScheduler();
	} /* !NULL */
	else
	{
		for(;;){};
	} /* NULL */
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void TASK_Run(void) {

	MOT_Init();
	RNET_Init();
	TACHO_Init();
	PID_Init();
	DRV_Init();

	TASK_AdoptToHardware();
	APPL_Init();
	TASK_CreateTasks();
}


/* Define a task that performs an action every x milliseconds. */
void TASK_PerdTaskFct(void * pvParameters_)
{
	uint8 i = 0u;
	const TASK_PerdTaskFctPar_t *pvPar = NULL;
	TickType_t LastWakeTime;

	pvPar = (const TASK_PerdTaskFctPar_t *)pvParameters_;

	/* The xLastWakeTime variable needs to be initialized with the current tick
	count.  Note that this is the only time the variable is written to explicitly.
	After this assignment, xLastWakeTime is update d automatically internally within
	vTaskDelayUntil(). */
	LastWakeTime = FRTOS1_xTaskGetTickCount();
	/* Enter the loop that defines the task behavior. */
	FRTOS1_vTaskDelay( pdMS_TO_TICKS( 100u ));
	for( ;; )
	{
		/* This task should execute every 50 milliseconds.  Time is measured
		in ticks. The pdMS_TO_TICKS macro is used to convert milliseconds
		into ticks. xLastWakeTime is automatically updated within vTaskDelayUntil()
		so is not explicitly updated by the task. */
		FRTOS1_vTaskDelayUntil( &LastWakeTime, pdMS_TO_TICKS( pvPar->taskPeriod ) );

		/* Perform the periodic actions here. */
		if(NULL != pvPar->mainFctCfg)
		{
			for(i = 0u; i < pvPar->numMainFcts; i++)
			{
				if(NULL != pvPar->mainFctCfg[i].mainFct)
				{
					pvPar->mainFctCfg[i].mainFct();
				}
			}
		}
	}
}

void TASK_NonPerdTaskFct(void *pvParameters_)
{
	uint8 i = 0u;
	const TASK_NonPerdTaskFctPar_t *pvPar = NULL;

	pvPar = (const TASK_NonPerdTaskFctPar_t *)pvParameters_;

	FRTOS1_vTaskDelay( pdMS_TO_TICKS( 100u ));
	for(;;) {

		/* Perform the periodic actions here. */
		if(NULL != pvPar->mainFctCfg)
		{
			for(i = 0u; i < pvPar->numMainFcts; i++)
			{
				if(NULL != pvPar->mainFctCfg[i].mainFct)
				{
					pvPar->mainFctCfg[i].mainFct();
				}
			}
		}
		FRTOS1_vTaskDelay( pdMS_TO_TICKS( pvPar->taskDelay) );
	}
}




#ifdef MASTER_task_C_
#undef MASTER_task_C_
#endif /* !MASTER_task_C_ */
