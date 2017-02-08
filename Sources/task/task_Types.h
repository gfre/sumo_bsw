/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file task_Types.h
 * 
 *==================================================================================================
 */


#ifndef TASK_TYPES_H_
#define TASK_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "projdefs.h"
#include "Platform.h"

#ifdef MASTER_task_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef TaskFunction_t TaskFctHdl_t;
typedef void * TaskHdl_t;

typedef enum TASK_SuspType_e
{
	 TASK_SUSP_NEVER = 0x00
	,TASK_SUSP_DEFAULT
}TASK_SuspType_t;

typedef struct TASK_CfgItm_s
{
	TaskFctHdl_t taskFctHdl;
	const char_t * const taskName;
	const uint16 stackDepth;
	void * const pvParameters;
	uint32 taskPriority;
	TaskHdl_t taskHdl;
	TASK_SuspType_t suspTask;
}TASK_CfgItm_t;

typedef struct TASK_Cfg_s
{
	TASK_CfgItm_t *tasks;
	const uint8 numTasks;
}TASK_Cfg_t;

typedef void (TASK_MainFct_t)(void);

typedef struct TASK_MainFctCfg_s
{
	const char_t * const swcName;
	TASK_MainFct_t * const mainFct;
}TASK_MainFctCfg_t;



typedef struct TASK_PerdTaskFctPar_s
{
	const uint8 taskPeriod;
	const TASK_MainFctCfg_t *mainFctCfg;
	const uint8 numMainFcts;

}TASK_PerdTaskFctPar_t;

typedef struct TASK_NonPerdTaskFctPar_s
{
	const uint8 taskDelay;
	const TASK_MainFctCfg_t *mainFctCfg;
	const uint8 numMainFcts;
}TASK_NonPerdTaskFctPar_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/

EXTERNAL_ void TASK_PerdTaskFct(void *pvParameters);

EXTERNAL_ void TASK_NonPerdTaskFct(void *pvParameters);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !TASK_TYPES_H_ */
