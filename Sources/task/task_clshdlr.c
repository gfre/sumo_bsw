/***********************************************************************************************//**
 * @file		task_clshdlr.c
 * @ingroup		task
 * @brief 		Implementation of the command line shell handler for the SWC @a Task
 *
 * This module implements the interface of the SWC @ref task which is addressed to
 * the SWC @ref sh. It introduces application specific commands for requests
 * of status information via command line shell (@b CLS).
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#define MASTER_task_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "task_clshdlr.h"
#include "task_api.h"
#include "task_cfg.h"
#include "FRTOS1.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8 TASK_PrintHelp(const CLS1_StdIOType *io_);
static uint8 TASK_PrintStatus(const CLS1_StdIOType *io_);
static void TASK_PrintCalledMainFcts(const CLS1_StdIOType *io_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8 TASK_PrintHelp(const CLS1_StdIOType *io_)
{
	CLS1_SendHelpStr((unsigned char*)"task", (unsigned char*)"Group of task commands\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows task help or status\r\n", io_->stdOut);
	return ERR_OK;
}

static uint8 TASK_PrintStatus(const CLS1_StdIOType *io_)
{
	CLS1_SendStatusStr((unsigned char*)"task", (unsigned char*)"\r\n", io_->stdOut);
	TASK_PrintCalledMainFcts(io_);
	return ERR_OK;
}

static void TASK_PrintCalledMainFcts(const CLS1_StdIOType *io_)
{
	uint8 i = 0u;
	uint8 j = 0u;
	uint8 buf[128] = {""};
	const TASK_Cfg_t *taskCfg = NULL;
	const TASK_PerdTaskFctPar_t *taskFctPar = NULL;
	const char_t *mainFctName = NULL;
	uint8 taskName[12] = {""};

	taskCfg = TASK_Get_TasksCfg();

	if(NULL != taskCfg)
	{
		for(i = 0u; i < taskCfg->numTasks; i++)
		{
			if(NULL != taskCfg->tasks)
			{
				taskFctPar = (const TASK_PerdTaskFctPar_t*)taskCfg->tasks[i].pvParameters;
				UTIL1_strcat(taskName, sizeof(taskName), "  ");
				if(NULL != (taskCfg->tasks[i].taskHdl))
				{
					UTIL1_strcat(taskName, sizeof(taskName), FRTOS1_pcTaskGetTaskName((taskCfg->tasks[i].taskHdl)));
					CLS1_SendStatusStr(taskName, (unsigned char*)"handle created\r\n", io_->stdOut);
				}
				else
				{
					UTIL1_strcat(taskName, sizeof(taskName), "NULL");
					CLS1_SendStatusStr(taskName, (unsigned char*)">> ERROR task handle not created <<\r\n", io_->stdOut);
				}
				UTIL1_strcpy(taskName, sizeof(taskName), "");

				taskFctPar = (const TASK_PerdTaskFctPar_t*)taskCfg->tasks[i].pvParameters;
				if(NULL != taskFctPar)
				{
					UTIL1_strcat(buf, sizeof(buf), ">> " );
					for(j = 0u; j < taskFctPar->numSwc; j++)
					{
						if(NULL != taskFctPar->swcCfg)
						{
							UTIL1_strcat(buf, sizeof(buf), taskFctPar->swcCfg[j].swcName);
							if(j < taskFctPar->numSwc-1u)
							{
								UTIL1_strcat(buf, sizeof(buf), ", ");
							}
						}

					}
					UTIL1_strcat(buf, sizeof(buf), "\r\n");
					CLS1_SendStatusStr((unsigned char*)"   runs", (unsigned char*)buf, io_->stdOut);
					UTIL1_strcpy(buf, sizeof(buf), "");
				}
			}
		}
	}
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t TASK_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	uint8_t res = ERR_OK;
	if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd_, (char*)"task help")==0) {
		*handled_ = TRUE;
		return TASK_PrintHelp(io_);
	} else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd_, (char*)"task status")==0) {
		*handled_ = TRUE;
		return TASK_PrintStatus(io_);
	}
	return res;
}



#ifdef MASTER_task_clshdlr_C_
#undef MASTER_task_clshdlr_C_

#endif /* !MASTER_task_clshdlr_C_ */
