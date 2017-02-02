/*******************************************************************************
 * @brief 	This is a brief description.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel 
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 	01.01.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This is the detailed description of the file CAU_Types.c
 * 
 * ==============================================================================
 */

#define MASTER_APPL_CFG_C_

#include "appl_cfg.h"
#include "Tacho.h"
#include "state.h"
#include "sh.h"
#include "rnet.h"
#include "drv.h"


/*======================================== >> MACROS << ==========================================*/
#define NUM_OF_TASKS              (sizeof(taskCfg)/sizeof(taskCfg[0]))
#define TASK_PERIOD_5MS     (5u)
#define TASK_PERIOD_10MS    (10u)
#define TASK_DELAY_10MS     (10u)

/*==================================== >> MODUL FUNCTUIONS << ====================================*/
/* Task functions for periodic tasks */
static void mainTaskFct(void *pvParameters_) {return APPL_PerdTaskFct(pvParameters_);}
static void rnetTaskFct(void *pvParameters_) {return APPL_PerdTaskFct(pvParameters_);}
static void drvTaskFct(void *pvParameters_)  {return APPL_PerdTaskFct(pvParameters_);}

/* Task functions for non-periodic tasks */
static void shTaskFct(void *pvParameters_)   {return APPL_NonPerdTaskFct(pvParameters_);}


/*=============================== >> MAIN FUNCTIONS CONFIGURATION << =============================*/
/*
 * Main function(s) for MAIN task
 */
const APPL_MainFctCfg_t mainTaskMainFctCfg[] = {
		{STATE_SWC_STRING, STATE_mainFct},
};

/*
 * Main function(s) for RNET task
 */
const APPL_MainFctCfg_t rnetTaskMainFctCfg[] = {
		{RNET_SWC_STRING, RNET_MainFct},

};

/*
 * Main function(s) for SHELL task
 */
const APPL_MainFctCfg_t shTaskMainFctCfg[] = {
		{SH_SWC_STRING, SH_MainFct},
};

/*
 * Main function(s) for SHELL task
 */
const APPL_MainFctCfg_t drvTaskMainFctCfg[] = {
		{DRV_SWC_STRING, DRV_MainFct},
		{TACHO_SWC_STRING, TACHO_CalcSpeed},
};


/*================================== >> TASK FUNCTIONS PARAMETERS << =============================*/
/*
 * MAIN task parameters
 */
const APPL_PerdTaskFctPar_t mainTaskFctPar = {
		TASK_PERIOD_10MS,
		mainTaskMainFctCfg,
		sizeof(mainTaskMainFctCfg)/sizeof(mainTaskMainFctCfg[0])
};

/*
 * RNET task parameters
 */
const APPL_PerdTaskFctPar_t rnetTaskFctPar = {
		TASK_PERIOD_5MS,
		rnetTaskMainFctCfg,
		sizeof(rnetTaskMainFctCfg)/sizeof(rnetTaskMainFctCfg[0])
};

/*
 * SHELL task parameters
 */
const APPL_NonPerdTaskFctPar_t shTaskFctPar = {
		TASK_DELAY_10MS,
		shTaskMainFctCfg,
		sizeof(shTaskMainFctCfg)/sizeof(shTaskMainFctCfg[0])
};

/*
 * DRIVE task parameters
 */
const APPL_PerdTaskFctPar_t drvTaskFctPar = {
		TASK_PERIOD_5MS,
		drvTaskMainFctCfg,
		sizeof(drvTaskMainFctCfg)/sizeof(drvTaskMainFctCfg[0])
};


/*============================ >> TASK CONFIGURATION << ===================================*/
/*
 * Configuration of each task in an array
 */
APPL_TaskCfgItm_t taskCfg[]= {
		{mainTaskFct, MAIN_TASK_STRING, configMINIMAL_STACK_SIZE,     (void *)&mainTaskFctPar, tskIDLE_PRIORITY+1, (xTaskHandle*)NULL, APPL_SUSP_NEVER},
		{shTaskFct,   SH_TASK_STRING,   configMINIMAL_STACK_SIZE+50,  (void *)&shTaskFctPar,   tskIDLE_PRIORITY+1, (xTaskHandle*)NULL, APPL_SUSP_DEFAULT},
		{rnetTaskFct, RNET_TASK_STRING, configMINIMAL_STACK_SIZE+100, (void *)&rnetTaskFctPar, tskIDLE_PRIORITY+3, (xTaskHandle*)NULL, APPL_SUSP_NEVER},
		{drvTaskFct,  DRV_TASK_STRING,  configMINIMAL_STACK_SIZE,     (void *)&drvTaskFctPar,  tskIDLE_PRIORITY+3, (xTaskHandle*)NULL, APPL_SUSP_NEVER,}
};

/*
 * Configuration summeray of all tasks
 */
const APPL_TaskCfg_t APPL_taskCfg = {
		taskCfg,
		NUM_OF_TASKS,
};


/*=============================== >> TASK CONFIGURATION INTERFACE << =============================*/

const APPL_TaskCfg_t *Get_APPL_TaskCfg(void) { return &APPL_taskCfg;}

const APPL_TaskCfgItm_t *Get_APPL_MainTaskCfg(void) { return &(APPL_taskCfg.tasks[0]);}
const APPL_TaskCfgItm_t *Get_APPL_ShTaskCfg(void)   { return &(APPL_taskCfg.tasks[1]);}

#ifdef MASTER_APPL_CFG_C_
#undef MASTER_APPL_CFG_C_
#endif
