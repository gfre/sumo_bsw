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
#include "KEY1.h"
#include "Tacho.h"
#include "state.h"


/* Macros */
#define NUM_OF_TASKS              (sizeof(taskCfg)/sizeof(taskCfg[0]))
#define TASK_CYCLE_PERIOD_10MS    (10u)
#define TASK_DELAY_10MS           (10u)

/* Task functions for cyclic tasks */
static void mainTaskFct(void *pvParameters_) {return APPL_CycTaskFct(pvParameters_);}

/* Task functions for non-cyclic tasks */
static void shTaskFct(void *pvParameters_)   {return APPL_NonCycTaskFct(pvParameters_);}

const APPL_MainFctCfg_t mainTaskMainFctCfg[] = {
		{TACHO_CalcSpeed, "tacho"},
		{STATE_mainFct, "state"}
};

const APPL_MainFctCfg_t shTaskMainFctCfg[] = {

};


const APPL_CycTaskFctPar_t mainTaskFctPar = {
		TASK_CYCLE_PERIOD_10MS,
		mainTaskMainFctCfg,
		sizeof(mainTaskMainFctCfg)/sizeof(mainTaskMainFctCfg[0])
};



const APPL_NonCycTaskFctPar_t shTaskFctPar = {
		TASK_DELAY_10MS,
		shTaskMainFctCfg,
		sizeof(shTaskMainFctCfg)/sizeof(shTaskMainFctCfg[0])
};

APPL_TaskCfgItm_t taskCfg[]= {
		{mainTaskFct, "MAIN",  configMINIMAL_STACK_SIZE,    (void *)&mainTaskFctPar, tskIDLE_PRIORITY+1, NULL},
		{shTaskFct,   "SHELL", configMINIMAL_STACK_SIZE+50, (void *)&shTaskFctPar,   tskIDLE_PRIORITY+1, NULL},
};

const APPL_TaskCfg_t APPL_taskCfg = {
		taskCfg,
		NUM_OF_TASKS,
};

const APPL_TaskCfg_t *Get_APPL_TaskCfg(void) { return &APPL_taskCfg;}



#ifdef MASTER_APPL_CFG_C_
#undef MASTER_APPL_CFG_C_
#endif
