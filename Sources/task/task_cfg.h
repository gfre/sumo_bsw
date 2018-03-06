/***********************************************************************************************//**
 * @file		task_cfg.h
 * @ingroup		task
 * @brief 		SWC-internal configuration interface of the SWC @a Task
 *
 * This header file provides an internal interface within the software component SWC @ref task
 * for the configuration of FreeRTOS tasks, which run periodically or non-periodically task
 * functions.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @note Interface for SWC-specific use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef TASK_CFG_H_
#define TASK_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "task_api.h"



#ifdef MASTER_task_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup task
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the task  @a APPLICATION
 */
#define APPL_TASK_STRING  ("APPL")

/**
 * String identification of the task  @a COMMUNICATION
 */
#define COMM_TASK_STRING  ("COMM")

/**
 * String identification of the task  @a DEBUG
 */
#define DBG_TASK_STRING   ("DEBUG")

/**
 * String identification of the task  @a DRIVE
 */
#define DRV_TASK_STRING   ("DRIVE")

/**
 * String identification of the task  @a REFL
 */
#define REFL_TASK_STRING   ("REFL")




/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @brief Data type definition of a main function of a basic software component
 */
typedef void (TASK_MainFct_t)(void);

/**
 * @brief Data type definition of an initialisation function of a basic software component
 * @param
 */
typedef void (TASK_InitFct_t)(const void *);

/**
 * @typedef TASK_SwcCfg_t
 * @brief Data type definition of the structure TASK_SwcCfg_s
 *
 * @struct TASK_SwcCfg_s
 * @brief This structure defines the properties of a software component
 */
typedef struct TASK_SwcCfg_s
{
	const char_t * const swcName;		/**< reference to the string representing the name of the SWC */
	TASK_MainFct_t * const mainFct;		/**< function handle to the main function of the SWC */
	TASK_InitFct_t * const initFct;		/**< function handle to the init function of the SWC */
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
	const uint8 taskPeriod;			/**< time period of the periodically task */
	const TASK_SwcCfg_t *swcCfg;	/**< reference to the (configuration)[@ref TASK_SwcCfg_t]  of the SWCs  */
	const uint8 numSwc;				/**< count of software components */
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
	const uint8 taskDelay;			/**< time delay of the periodically task */
	const TASK_SwcCfg_t *swcCfg;	/**< reference to the (configuration)[@ref TASK_SwcCfg_t]  of the SWCs  */
	const uint8 numSwc;				/**< count of software components */
}TASK_NonPerdTaskFctPar_t;

/**
 * @brief Data type (re-)definition of a task function handle inherited from TaskFunction_t defined
 * in projdefs.h by FreeRTOS.
 */
typedef TaskFunction_t TASK_FctHdl_t;

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
	const TASK_FctHdl_t taskFctHdl;		/**< function handle of the task function */
	const char_t * const taskName;		/**< reference to the string representing the task name */
	const uint16 stackDepth;			/**< stack depth for stack memory allocation */
	void * const pvParameters;			/**< reference to the parameters passed into the task */
	uint32 taskPriority;				/**< task priority */
	TASK_Hdl_t taskHdl;					/**< handle to the task object */
	const TASK_SuspType_t suspTask;		/**< see @ref TASK_SuspType_e */
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
	TASK_CfgItm_t *tasks;		/**< reference to the task configuration table */
	uint8_t numTasks;			/**< count of task configuration items */
}TASK_Cfg_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Function returns the entire configuration of all tasks
 * @return pointer to the entire task configuration
 */
EXTERNAL_ const TASK_Cfg_t *Get_pTaskCfgTbl(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TASK_CFG_H_ */
