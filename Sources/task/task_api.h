/***********************************************************************************************//**
 * @file		task_api.h
 * @ingroup		task
 * @brief 		API of the SWC @a Task
 *
 * This API provides a BSW-internal interface of the SWC @ref task. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef TASK_TYPES_H_
#define TASK_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "FRTOS1.h"
#include "Platform.h"



#ifdef MASTER_task_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup task
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @brief Data type (re-)definition of a task function handle inherited from TaskFunction_t defined
 * in projdefs.h by FreeRTOS.
 */
typedef TaskFunction_t TaskFctHdl_t;

/**
 * @brief Data type (re-)definition of a task handle inherited from TaskHandle_t defined in task.h
 * by FreeRTOS.
 */
typedef TaskHandle_t TaskHdl_t;

/**
 * @typedef TASK_SuspType_t
 * @brief Data type definition of the enumeration TASK_SuspType_e
 *
 * @enum TASK_SuspType_e
 * @brief This enumeration defines the possibility of suspension of a task
 */
typedef enum TASK_SuspType_e
{
	 TASK_SUSP_NEVER = 0x00		  	/**< task will be never suspended */
	,TASK_SUSP_DEFAULT				/**< task is suspended at default */
}TASK_SuspType_t;

/**
 * @typedef TASK_CfgItm_t
 * @brief Data type definition of the structure TASK_CfgItm_s
 *
 * @struct TASK_CfgItm_s
 * @brief This structure defines the properties of a task
 */
typedef struct TASK_CfgItm_s
{
	TaskFctHdl_t taskFctHdl;		/**< function handle of the task function */
	const char_t * const taskName;	/**< reference to the string representing the task name */
	const uint16 stackDepth;		/**< stack depth for stack memory allocation */
	void * const pvParameters;		/**< reference to the parameters passed into the task */
	uint32 taskPriority;			/**< task priority */
	TaskHdl_t taskHdl;				/**< handle to the task object */
	TASK_SuspType_t suspTask;		/**< see @ref TASK_SuspType_e */
}TASK_CfgItm_t;

/**
 * @typedef TASK_Cfg_t
 * @brief Data type definition of the structure TASK_Cfg_s
 *
 * @struct TASK_Cfg_s
 * @brief This structure defines a configuration table which holds all task configuration items
 */
typedef struct TASK_Cfg_s
{
	TASK_CfgItm_t *tasks;			/**< reference to the task configuration table */
	const uint8 numTasks;			/**< count of task configuration items */
}TASK_Cfg_t;

/**
 * @brief Data type definition of a main function of a basic software component
 */
typedef void (TASK_MainFct_t)(void);

/**
 * @brief Data type definition of an initialisation function of a basic software component
 */
typedef void (TASK_InitFct_t)(void);

/**
 * @typedef TASK_SwcCfg_t
 * @brief Data type definition of the structure TASK_SwcCfg_s
 *
 * @struct TASK_SwcCfg_s
 * @brief This structure defines the properties of a software component
 */
typedef struct TASK_SwcCfg_s
{
	const char_t * const swcName;	/**< reference to the string representing the name of the SWC */
	TASK_MainFct_t * const mainFct;	/**< function handle to the main function of the SWC */
	TASK_InitFct_t * const initFct;	/**< function handle to the init function of the SWC */
}TASK_SwcCfg_t;

/**
 * @typedef TASK_PerdTaskFctPar_t
 * @brief Data type definition of the structure TASK_PerdTaskFctPar_s
 *
 * @struct TASK_PerdTaskFctPar_s
 * @brief This structure holds the [pvParameters](@ref TASK_CfgItm_s) of a periodically called task
 * which are passed into the task via its task function.
 */
typedef struct TASK_PerdTaskFctPar_s
{
	const uint8 taskPeriod;			/**< */
	const TASK_SwcCfg_t *swcCfg;	/**< */
	const uint8 numSwc;				/**< */
}TASK_PerdTaskFctPar_t;

/**
 * @typedef TASK_NonPerdTaskFctPar_t
 * @brief Data type definition of the structure TASK_NonPerdTaskFctPar_s
 *
 * @struct TASK_NonPerdTaskFctPar_s
 * @brief This structure holds the [pvParameters](@ref TASK_CfgItm_s) of a non-periodically called
 * task which are passed into the task via its task function.
 */
typedef struct TASK_NonPerdTaskFctPar_s
{
	const uint8 taskDelay;			/**< */
	const TASK_SwcCfg_t *swcCfg;	/**< */
	const uint8 numSwc;				/**< */
}TASK_NonPerdTaskFctPar_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This is the generic task function of periodically called task
 * @param pvParameters
 */
EXTERNAL_ void TASK_PerdTaskFct(void *pvParameters);

/**
 * @brief This is the generic task function of non-periodically called task
 * @param pvParameters
 */
EXTERNAL_ void TASK_NonPerdTaskFct(void *pvParameters);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TASK_TYPES_H_ */
