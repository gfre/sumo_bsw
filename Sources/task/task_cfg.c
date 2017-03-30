/***************************************************************************************************
 * @brief 	This module configures all tasks
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file task_cfg.h
 * 
 *==================================================================================================
 */

#define MASTER_task_cfg_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "task_cfg.h"
#include "FRTOS1.h"
#include "tacho.h"
#include "appl.h"
#include "sh.h"
#include "rnet.h"
#include "drv.h"


/*======================================= >> #DEFINES << =========================================*/
#define NUM_OF_TASKS        (sizeof(taskCfgItems)/sizeof(taskCfgItems[0]))
#define TASK_TIMING_5MS     (5u)
#define TASK_TIMING_10MS    (10u)


#define APPL_TASK_PERIOD 	(TASK_TIMING_10MS)
#define COMM_TASK_PERIOD 	(TASK_TIMING_5MS)
#define DRV_TASK_PERIOD 	(TASK_TIMING_5MS)
#define DBG_TASK_DELAY 		(TASK_TIMING_10MS)

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
/* Task functions for periodic tasks */
static void ApplTaskFct(void *pvParameters_) {return TASK_PerdTaskFct(pvParameters_);}
static void CommTaskFct(void *pvParameters_) {return TASK_PerdTaskFct(pvParameters_);}
static void DrvTaskFct(void *pvParameters_)  {return TASK_PerdTaskFct(pvParameters_);}

/* Task functions for non-periodic tasks */
static void DbgTaskFct(void *pvParameters_)   {return TASK_NonPerdTaskFct(pvParameters_);}


/*=================================== >> GLOBAL VARIABLES << =====================================*/
/*
 * Configuration of the software component(s) run by the APPLICATION task
 */
const TASK_SwcCfg_t applTaskSwcCfg[] = {
		{APPL_SWC_STRING, APPL_MainFct, APPL_Init},
};

/*
 * Configuration of the software component(s) run by the COMMUNICATION task
 */
const TASK_SwcCfg_t commTaskSwcCfg[] = {
		{RNET_SWC_STRING, RNET_MainFct, RNET_Init},

};

/*
 * Configuration of the software component(s) run by the DEBUG task
 */
const TASK_SwcCfg_t dbgTaskSwcCfg[] = {
		{SH_SWC_STRING, SH_MainFct, SH_Init},
};

/*
 * Configuration of the software component(s) run by the DRIVE task
 */
const TASK_SwcCfg_t drvTaskSwcCfg[] = {
		{DRV_SWC_STRING, DRV_MainFct, DRV_Init},
		{TACHO_SWC_STRING, TACHO_CalcSpeed, TACHO_Init},
};
/*------------------------------------------------------------------------------------------------*/


/*
 * APPLICATION task parameters
 */
const TASK_PerdTaskFctPar_t applTaskFctPar = {
		APPL_TASK_PERIOD,
		applTaskSwcCfg,
		sizeof(applTaskSwcCfg)/sizeof(applTaskSwcCfg[0])
};

/*
 * COMMUNICATION task parameters
 */
const TASK_PerdTaskFctPar_t commTaskFctPar = {
		COMM_TASK_PERIOD,
		commTaskSwcCfg,
		sizeof(commTaskSwcCfg)/sizeof(commTaskSwcCfg[0])
};

/*
 * DEBUG task parameters
 */
const TASK_NonPerdTaskFctPar_t dbgTaskFctPar = {
		DBG_TASK_DELAY,
		dbgTaskSwcCfg,
		sizeof(dbgTaskSwcCfg)/sizeof(dbgTaskSwcCfg[0])
};

/*
 * DRIVE task parameters
 */
const TASK_PerdTaskFctPar_t drvTaskFctPar = {
		DRV_TASK_PERIOD,
		drvTaskSwcCfg,
		sizeof(drvTaskSwcCfg)/sizeof(drvTaskSwcCfg[0])
};
/*------------------------------------------------------------------------------------------------*/


/*
 * Configuration of each task in an array
 */
TASK_CfgItm_t taskCfgItems[]= {
		{ApplTaskFct, APPL_TASK_STRING, configMINIMAL_STACK_SIZE,     (void * const)&applTaskFctPar, tskIDLE_PRIORITY+1, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
		{DbgTaskFct,  DBG_TASK_STRING,  configMINIMAL_STACK_SIZE+50,  (void * const)&dbgTaskFctPar,  tskIDLE_PRIORITY+1, (xTaskHandle*)NULL, TASK_SUSP_DEFAULT},
		{CommTaskFct, COMM_TASK_STRING, configMINIMAL_STACK_SIZE+100, (void * const)&commTaskFctPar, tskIDLE_PRIORITY+3, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
		{DrvTaskFct,  DRV_TASK_STRING,  configMINIMAL_STACK_SIZE,     (void * const)&drvTaskFctPar,  tskIDLE_PRIORITY+3, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
};
/*------------------------------------------------------------------------------------------------*/


/*
 * Configuration summeray of all tasks
 */
const TASK_Cfg_t taskCfg = {
		taskCfgItems,
		NUM_OF_TASKS,
};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
const TASK_Cfg_t *TASK_Get_TasksCfg(void) { return &taskCfg;}

const TASK_CfgItm_t *TASK_Get_ApplTaskCfg(void) { return &(taskCfg.tasks[0]);}
const TASK_CfgItm_t *TASK_Get_DbgTaskCfg(void)  { return &(taskCfg.tasks[1]);}



#ifdef MASTER_task_cfg_C_
#undef MASTER_task_cfg_C_
#endif /* !MASTER_task_cfg_C_ */
