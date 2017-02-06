/*******************************************************************************
 * @brief 	Main Application Interface.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date		02.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * Interface to the main application module. From here the application
 * runs and performs all tasks.
 *
 * ==============================================================================
 */

#define MASTER_APPL_C_

#include "Platform.h"
#include "appl.h"
#include "appl_Types.h"
#include "appl_cfg.h"
#include "FRTOS1.h"
#include "WAIT1.h"
#include "sh.h"
#include "LED1.h"
#include "LED2.h"
#include "Buzzer.h"
#include "KEY1.h"
#include "Motor.h"
#include "RNET1.h"
#include "batt.h"
#include "Tacho.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "Pid.h"
#include "drv.h"
#include "nvm.h"
#include "id.h"
#include "rte.h"
#include "rnet.h"

static bool shEnable = FALSE; /* DEFAULT: Shell is off */

static void APPL_TaskCreate();
static uint8_t APPL_PrintHelp(const CLS1_StdIOType *io);
static uint8_t APPL_PrintStatus(const CLS1_StdIOType *io);
static void APPL_PrintCalledMainFcts(const CLS1_StdIOType *io_);


static uint8_t APPL_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*)"appl", (unsigned char*)"Group of appl commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows appl help or status\r\n", io->stdOut);
	return ERR_OK;
}

static uint8_t APPL_PrintStatus(const CLS1_StdIOType *io) {
	CLS1_SendStatusStr((unsigned char*)"appl", (unsigned char*)"\r\n", io->stdOut);
	APPL_PrintCalledMainFcts(io);
	return ERR_OK;
}

static void APPL_PrintCalledMainFcts(const CLS1_StdIOType *io_)
{
	uint8 i = 0u;
	uint8 j = 0u;
	uint8 buf[128] = {""};
	const APPL_TaskCfg_t *taskCfg = NULL;
	const APPL_PerdTaskFctPar_t *taskFctPar = NULL;
	const char_t *mainFctName = NULL;
	uint8 taskName[12] = {""};

	taskCfg = Get_APPL_TaskCfg();

	if(NULL != taskCfg)
	{
		for(i = 0u; i < taskCfg->numTasks; i++)
		{
			if(NULL != taskCfg->tasks)
			{
				taskFctPar = (const APPL_PerdTaskFctPar_t*)taskCfg->tasks[i].pvParameters;
				UTIL1_strcat(taskName, sizeof(taskName), "  ");
				if(NULL != (taskCfg->tasks[i].taskHdl))
				{
					UTIL1_strcat(taskName, sizeof(taskName), FRTOS1_pcTaskGetTaskName((taskCfg->tasks[i].taskHdl)));
					CLS1_SendStatusStr(taskName, (unsigned char*)"task handle successfully created\r\n", io_->stdOut);
				}
				else
				{
					UTIL1_strcat(taskName, sizeof(taskName), "NULL");
					CLS1_SendStatusStr(taskName, (unsigned char*)">> ERROR task handle not created <<\r\n", io_->stdOut);
				}
				UTIL1_strcpy(taskName, sizeof(taskName), "");

				taskFctPar = (const APPL_PerdTaskFctPar_t*)taskCfg->tasks[i].pvParameters;
				if(NULL != taskFctPar)
				{
					UTIL1_strcat(buf, sizeof(buf), ">> " );
					for(j = 0u; j < taskFctPar->numMainFcts; j++)
					{
						if(NULL != taskFctPar->mainFctCfg)
						{
							UTIL1_strcat(buf, sizeof(buf), taskFctPar->mainFctCfg[j].swcName);
							if(j < taskFctPar->numMainFcts-1u)
							{
								UTIL1_strcat(buf, sizeof(buf), ", ");
							}
						}

					}
					UTIL1_strcat(buf, sizeof(buf), "\r\n");
					CLS1_SendStatusStr((unsigned char*)"   sw comps", (unsigned char*)buf, io_->stdOut);
					UTIL1_strcpy(buf, sizeof(buf), "");
				}
			}
		}
	}
}

static void APPL_AdoptToHardware(void) {
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

static void APPL_TaskCreate()
{
	uint8 i = 0u;
	const APPL_TaskCfg_t *taskCfg = NULL;
	taskCfg = Get_APPL_TaskCfg();

	if(NULL != taskCfg)
	{
		for(i = 0u; i < taskCfg->numTasks; i++)
		{
			if(NULL != taskCfg->tasks)
			{
				if(pdPASS != FRTOS1_xTaskCreate(taskCfg->tasks[i].taskFct,
						taskCfg->tasks[i].taskName,
						taskCfg->tasks[i].stackDepth,
						taskCfg->tasks[i].pvParameters,
						taskCfg->tasks[i].taskPriority,
					   &taskCfg->tasks[i].taskHdl))
				{
					/* The task could not be created because there was not enough
								FreeRTOS heap memory available for the task data structures and
								stack to be allocated. */
				}
				else
				{
					/* The task was created successfully */
					if(APPL_SUSP_DEFAULT == taskCfg->tasks[i].suspTask)
					{
						FRTOS1_vTaskSuspend(taskCfg->tasks[i].taskHdl);
					}
				}
			}
		}
		FRTOS1_vTaskStartScheduler();
	}
	else
	{
		for(;;){};
	}
}


static void APPL_Init(void){
	APPL_TaskCreate();
}


uint8_t APPL_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	uint8_t res = ERR_OK;
	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"appl help")==0) {
		*handled = TRUE;
		return APPL_PrintHelp(io);
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"appl status")==0) {
		*handled = TRUE;
		return APPL_PrintStatus(io);
	}
	return res;
}

void APPL_Run(void) {

	BUZ_Init();
	MOT_Init();
	RNET1_Init();
	BATT_Init();
	TACHO_Init();
	PID_Init();
	DRV_Init();
	ID_Init();
	NVM_Init();

	APPL_AdoptToHardware();

	RNET_Init();
    APPL_Init();
}


/* Define a task that performs an action every x milliseconds. */
void APPL_PerdTaskFct(void * pvParameters_)
{
	uint8 i = 0u;
	const APPL_PerdTaskFctPar_t *pvPar = NULL;
	TickType_t LastWakeTime;

	pvPar = (const APPL_PerdTaskFctPar_t *)pvParameters_;

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

void APPL_NonPerdTaskFct(void *pvParameters_)
{
	uint8 i = 0u;
	const APPL_NonPerdTaskFctPar_t *pvPar = NULL;

	pvPar = (const APPL_NonPerdTaskFctPar_t *)pvParameters_;

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


#ifdef MASTER_APPL_C_
#undef MASTER_APPL_C_
#endif
