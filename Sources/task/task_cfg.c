/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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
#include "Tacho.h"
#include "state.h"
#include "sh.h"
#include "rnet.h"
#include "drv.h"



/*======================================= >> #DEFINES << =========================================*/
#define NUM_OF_TASKS        (sizeof(taskCfgItems)/sizeof(taskCfgItems[0]))
#define TASK_PERIOD_5MS     (5u)
#define TASK_PERIOD_10MS    (10u)
#define TASK_DELAY_10MS     (10u)


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
/* Task functions for periodic tasks */
static void mainTaskFct(void *pvParameters_) {return TASK_PerdTaskFct(pvParameters_);}
static void rnetTaskFct(void *pvParameters_) {return TASK_PerdTaskFct(pvParameters_);}
static void drvTaskFct(void *pvParameters_)  {return TASK_PerdTaskFct(pvParameters_);}

/* Task functions for non-periodic tasks */
static void shTaskFct(void *pvParameters_)   {return TASK_NonPerdTaskFct(pvParameters_);}


/*=================================== >> GLOBAL VARIABLES << =====================================*/
/*
 * Main function(s) for MAIN task
 */
const TASK_MainFctCfg_t mainTaskMainFctCfg[] = {
		{STATE_SWC_STRING, STATE_mainFct},
};

/*
 * Main function(s) for RNET task
 */
const TASK_MainFctCfg_t rnetTaskMainFctCfg[] = {
		{RNET_SWC_STRING, RNET_MainFct},

};

/*
 * Main function(s) for SHELL task
 */
const TASK_MainFctCfg_t shTaskMainFctCfg[] = {
		{SH_SWC_STRING, SH_MainFct},
};

/*
 * Main function(s) for SHELL task
 */
const TASK_MainFctCfg_t drvTaskMainFctCfg[] = {
		{DRV_SWC_STRING, DRV_MainFct},
		{TACHO_SWC_STRING, TACHO_CalcSpeed},
};
/*------------------------------------------------------------------------------------------------*/


/*
 * MAIN task parameters
 */
const TASK_PerdTaskFctPar_t mainTaskFctPar = {
		TASK_PERIOD_10MS,
		mainTaskMainFctCfg,
		sizeof(mainTaskMainFctCfg)/sizeof(mainTaskMainFctCfg[0])
};

/*
 * RNET task parameters
 */
const TASK_PerdTaskFctPar_t rnetTaskFctPar = {
		TASK_PERIOD_5MS,
		rnetTaskMainFctCfg,
		sizeof(rnetTaskMainFctCfg)/sizeof(rnetTaskMainFctCfg[0])
};

/*
 * SHELL task parameters
 */
const TASK_NonPerdTaskFctPar_t shTaskFctPar = {
		TASK_DELAY_10MS,
		shTaskMainFctCfg,
		sizeof(shTaskMainFctCfg)/sizeof(shTaskMainFctCfg[0])
};

/*
 * DRIVE task parameters
 */
const TASK_PerdTaskFctPar_t drvTaskFctPar = {
		TASK_PERIOD_5MS,
		drvTaskMainFctCfg,
		sizeof(drvTaskMainFctCfg)/sizeof(drvTaskMainFctCfg[0])
};
/*------------------------------------------------------------------------------------------------*/


/*
 * Configuration of each task in an array
 */
TASK_CfgItm_t taskCfgItems[]= {
		{mainTaskFct, MAIN_TASK_STRING, configMINIMAL_STACK_SIZE,     (void *)&mainTaskFctPar, tskIDLE_PRIORITY+1, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
		{shTaskFct,   SH_TASK_STRING,   configMINIMAL_STACK_SIZE+50,  (void *)&shTaskFctPar,   tskIDLE_PRIORITY+1, (xTaskHandle*)NULL, TASK_SUSP_DEFAULT},
		{rnetTaskFct, RNET_TASK_STRING, configMINIMAL_STACK_SIZE+100, (void *)&rnetTaskFctPar, tskIDLE_PRIORITY+3, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
		{drvTaskFct,  DRV_TASK_STRING,  configMINIMAL_STACK_SIZE,     (void *)&drvTaskFctPar,  tskIDLE_PRIORITY+3, (xTaskHandle*)NULL, TASK_SUSP_NEVER},
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
const TASK_Cfg_t *Get_TASK_Cfg(void) { return &taskCfg;}

const TASK_CfgItm_t *Get_TASK_MainTaskCfg(void) { return &(taskCfg.tasks[0]);}
const TASK_CfgItm_t *Get_TASK_ShTaskCfg(void)   { return &(taskCfg.tasks[1]);}



#ifdef MASTER_task_cfg_C_
#undef MASTER_task_cfg_C_
#endif /* !MASTER_task_cfg_C_ */
