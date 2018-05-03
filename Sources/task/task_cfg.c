/***********************************************************************************************//**
 * @file		task_cfg.c
 * @ingroup		task
 * @brief 		Implementation of the configuration of the SWC @a Task
 *
 * This file implements the configuration of FreeRTOS tasks, which run periodically or non-
 * periodically task functions and an internal interface for the entire task configuration.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#define MASTER_task_cfg_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "task_cfg.h"
#include "tacho.h"
#include "appl.h"
#include "sh.h"
#include "rnet.h"
#include "drv.h"
#include "refl.h"


/*======================================= >> #DEFINES << =========================================*/
#define NUM_OF_TASKS        (sizeof(taskCfgItems)/sizeof(taskCfgItems[0]))
#define TASK_TIMING_1MS     (1u)
#define TASK_TIMING_2MS     (2u)
#define TASK_TIMING_3MS     (3u)
#define TASK_TIMING_5MS     (5u)
#define TASK_TIMING_10MS    (10u)
#define TASK_TIMING_20MS	(20u)

#define APPL_TASK_PERIOD 	(TASK_TIMING_10MS)
#define COMM_TASK_PERIOD 	(TASK_TIMING_5MS)
#define DRV_TASK_PERIOD 	(TASK_TIMING_5MS)
#define DBG_TASK_DELAY 		(TASK_TIMING_10MS)
#define REFL_TASK_DELAY		(TASK_TIMING_20MS)

/* Task functions for periodic tasks */
#define APPL_TASKFCT		(TASK_PerdTaskFct)
#define COMM_TASKFCT		(TASK_PerdTaskFct)
#define DRV_TASKFCT			(TASK_PerdTaskFct)

/* Task functions for non-periodic tasks */
#define DBG_TASKFCT			(TASK_NonPerdTaskFct)
#define	REFL_TASKFCT		(TASK_NonPerdTaskFct)


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
/*
 * Configuration of the software component(s) run by the APPLICATION task
 */
static const TASK_SwcCfg_t applTaskSwcCfg[] = {
		{APPL_SWC_STRING, APPL_MainFct, APPL_Init},
};

/*
 * Configuration of the software component(s) run by the COMMUNICATION task
 */
static const TASK_SwcCfg_t commTaskSwcCfg[] = {
		{RNET_SWC_STRING, RNET_MainFct, RNET_Init},

};

/*
 * Configuration of the software component(s) run by the DEBUG task
 */
static const TASK_SwcCfg_t dbgTaskSwcCfg[] = {
		{SH_SWC_STRING, SH_MainFct, SH_Init},
};

/*
 * Configuration of the software component(s) run by the DRIVE task
 */
static const TASK_SwcCfg_t drvTaskSwcCfg[] = {
		{DRV_SWC_STRING, DRV_MainFct, DRV_Init},
		{TACHO_SWC_STRING, TACHO_Main, TACHO_Init},
};

/*
 * Configuration of the software component(s) run by the DRIVE task
 */
static const TASK_SwcCfg_t reflTaskSwcCfg[] = {
		{REFL_SWC_STRING, REFL_MainFct, REFL_Init},
};
/*------------------------------------------------------------------------------------------------*/


/*
 * APPLICATION task parameters
 */
static const TASK_PerdTaskFctPar_t applTaskFctPar = {
		APPL_TASK_PERIOD,
		applTaskSwcCfg,
		sizeof(applTaskSwcCfg)/sizeof(applTaskSwcCfg[0])
};

/*
 * COMMUNICATION task parameters
 */
static const TASK_PerdTaskFctPar_t commTaskFctPar = {
		COMM_TASK_PERIOD,
		commTaskSwcCfg,
		sizeof(commTaskSwcCfg)/sizeof(commTaskSwcCfg[0])
};

/*
 * DEBUG task parameters
 */
static const TASK_NonPerdTaskFctPar_t dbgTaskFctPar = {
		DBG_TASK_DELAY,
		dbgTaskSwcCfg,
		sizeof(dbgTaskSwcCfg)/sizeof(dbgTaskSwcCfg[0])
};

/*
 * DRIVE task parameters
 */
static const TASK_PerdTaskFctPar_t drvTaskFctPar = {
		DRV_TASK_PERIOD,
		drvTaskSwcCfg,
		sizeof(drvTaskSwcCfg)/sizeof(drvTaskSwcCfg[0])
};

/*
 * REFL task parameters
 */
static const TASK_PerdTaskFctPar_t reflTaskFctPar = {
		REFL_TASK_DELAY,
		reflTaskSwcCfg,
		sizeof(reflTaskSwcCfg)/sizeof(reflTaskSwcCfg[0])
};
/*------------------------------------------------------------------------------------------------*/


/*
 * Configuration of each task in an array
 */
static TASK_CfgItm_t taskCfgItems[]= {
		{APPL_TASKFCT, APPL_TASK_STRING, configMINIMAL_STACK_SIZE,     (void * const)&applTaskFctPar, tskIDLE_PRIORITY+2, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
		{DBG_TASKFCT,  DBG_TASK_STRING,  configMINIMAL_STACK_SIZE+50,  (void * const)&dbgTaskFctPar,  tskIDLE_PRIORITY+1, (xTaskHandle*)NULL, TASK_SUSP_DEFAULT},
		{COMM_TASKFCT, COMM_TASK_STRING, configMINIMAL_STACK_SIZE+100, (void * const)&commTaskFctPar, tskIDLE_PRIORITY+3, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
		{DRV_TASKFCT,  DRV_TASK_STRING,  configMINIMAL_STACK_SIZE,     (void * const)&drvTaskFctPar,  tskIDLE_PRIORITY+3, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
//		{REFL_TASKFCT, REFL_TASK_STRING, configMINIMAL_STACK_SIZE+50,  (void * const)&reflTaskFctPar, tskIDLE_PRIORITY+4, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
};
/*------------------------------------------------------------------------------------------------*/


/*
 * Configuration summeray of all tasks
 */
static const TASK_Cfg_t taskCfg = {
		taskCfgItems,
		NUM_OF_TASKS,
};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
const TASK_Cfg_t *Get_pTaskCfgTbl(void) { return &taskCfg;}





#ifdef MASTER_task_cfg_C_
#undef MASTER_task_cfg_C_
#endif /* !MASTER_task_cfg_C_ */
