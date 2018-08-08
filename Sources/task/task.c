/***********************************************************************************************//**
 * @file		task.c
 * @ingroup		task
 * @brief 		Implementation of FreeRTOS tasks creation and handling
 *
 * This software component creates and handles the FreeRTOS tasks. It implements generic task
 * functions for periodically and non-periodically runtime-calls of software components applied to
 * the corresponding task. It enables to initialises the software components run by the created
 * tasks before entering the runtime loop.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#define MASTER_task_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "task.h"
#include "task_cfg.h"
#include "task_api.h"
#include "mot.h"
#include "RNET1.h"
#include "tacho.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "drv.h"
#include "rnet.h"
#include "appl.h"
#include "refl.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void TASK_CreateTasks(void);
static StdRtn_t ReadTaskHdl(TASK_Hdl_t *hdl_, const char_t * pName_);
static StdRtn_t ReadTaskPeriod(uint8_t *taskPer_, const char_t * pName_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
const TASK_Cfg_t *taskCfg = NULL;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void TASK_CreateTasks()
{
	uint8 i = 0u;

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
	} /* !NULL */
	else
	{
		for(;;){};
	} /* NULL */
}

StdRtn_t ReadTaskHdl(TASK_Hdl_t *hdl_, const char_t * pName_)
{
	StdRtn_t retVal = ERR_PARAM_DATA;
	uint8_t i = 0u;
	if( NULL != taskCfg )
	{
		if( NULL != hdl_)
		{
			for( i = 0u; i < taskCfg->numTasks; i++)
			{
				if( ( ERR_OK == UTIL1_strcmp(pName_,	taskCfg->tasks[i].taskName) ) && ( NULL != taskCfg->tasks[i].taskHdl ) )
				{
					*hdl_ = taskCfg->tasks[i].taskHdl;
					retVal = ERR_OK;
				}
			}
		}
		else
		{
			retVal = ERR_PARAM_ADDRESS;
		}
	}
	return retVal;
}


StdRtn_t ReadTaskPeriod(uint8_t *taskPer_, const char_t * pName_)
{
	StdRtn_t retVal = ERR_PARAM_DATA;
	uint8_t i = 0u;
	if( NULL != taskCfg )
	{
		if( NULL != taskPer_)
		{
			for( i = 0u; i < taskCfg->numTasks; i++)
			{
				if( ERR_OK == UTIL1_strcmp(pName_,	taskCfg->tasks[i].taskName) )
				{
					*taskPer_ = ((TASK_PerdTaskFctPar_t *)taskCfg->tasks[i].pvParameters)->taskPeriod;
					retVal = ERR_OK;
				}
			}
		}
		else
		{
			retVal = ERR_PARAM_ADDRESS;
		}
	}
	return retVal;
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void TASK_Init(void) {
	taskCfg = Get_pTaskCfgTbl();
	TASK_CreateTasks();
}


/* Define a task that performs an action every x milliseconds. */
void TASK_PerdTaskFct(void * pvParameters_)
{
	uint8 i = 0u;
	const TASK_PerdTaskFctPar_t *pvPar = NULL;
	TickType_t LastWakeTime = 0u;

	pvPar = (const TASK_PerdTaskFctPar_t *)pvParameters_;

	/* The xLastWakeTime variable needs to be initialized with the current tick
	count.  Note that this is the only time the variable is written to explicitly.
	After this assignment, xLastWakeTime is update d automatically internally within
	vTaskDelayUntil(). */
	LastWakeTime = FRTOS1_xTaskGetTickCount();

	/* Run initialisation before entering the loop */
	if( (NULL != pvPar) && (NULL != pvPar->swcCfg) )
	{
		for(i = 0u; i < pvPar->numSwc; i++)
		{
			if( NULL != pvPar->swcCfg[i].initFct )
			{
				pvPar->swcCfg[i].initFct((const void *)&pvPar->taskPeriod);
			}
		}
	}
	/* Enter the loop that defines the task behavior. */
	FRTOS1_vTaskDelay( pdMS_TO_TICKS( 100u ));
	for( ;; )
	{
		/* This task should execute every x milliseconds.  Time is measured
		in ticks. The pdMS_TO_TICKS macro is used to convert milliseconds
		into ticks. xLastWakeTime is automatically updated within vTaskDelayUntil()
		so is not explicitly updated by the task. */
		FRTOS1_vTaskDelayUntil( &LastWakeTime, pdMS_TO_TICKS( pvPar->taskPeriod ) );

		/* Perform the periodic actions here. */
		if( (NULL != pvPar) && (NULL != pvPar->swcCfg) )
		{
			for(i = 0u; i < pvPar->numSwc; i++)
			{
				if(NULL != pvPar->swcCfg[i].mainFct)
				{
					pvPar->swcCfg[i].mainFct();
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

	/* Run initialisation before entering the loop */
	if( (NULL != pvPar) && (NULL != pvPar->swcCfg) )
	{
		for(i = 0u; i < pvPar->numSwc; i++)
		{
			if( NULL != pvPar->swcCfg[i].initFct )
			{
				pvPar->swcCfg[i].initFct((const void *)&pvPar->taskDelay);
			}
		}
	}

	/* Enter the loop that defines the task behavior. */
	FRTOS1_vTaskDelay( pdMS_TO_TICKS( 100u ));
	for(;;) {

		/* Perform the periodic actions here. */
		if( (NULL != pvPar) && (NULL != pvPar->swcCfg) )
		{
			for(i = 0u; i < pvPar->numSwc; i++)
			{
				if(NULL != pvPar->swcCfg[i].mainFct)
				{
					pvPar->swcCfg[i].mainFct();
				}
			}
		}
		FRTOS1_vTaskDelay( pdMS_TO_TICKS( pvPar->taskDelay) );
	}
}


StdRtn_t TASK_Read_ApplTaskHdl(TASK_Hdl_t *hdl_)
{
	return ReadTaskHdl(hdl_, APPL_TASK_STRING);
}


StdRtn_t TASK_Read_DbgTaskHdl(TASK_Hdl_t *hdl_)
{
	return ReadTaskHdl(hdl_, DBG_TASK_STRING);
}

StdRtn_t TASK_Read_ApplTaskPeriod(uint8_t *taskPer_)
{
	return ReadTaskPeriod(taskPer_, APPL_TASK_STRING);
}


#ifdef MASTER_task_C_
#undef MASTER_task_C_

#endif /* !MASTER_task_C_ */
