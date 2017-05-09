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
#include "ACon_Types.h"



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
 * @brief Data type (re-)definition of a task handle inherited from TaskHandle_t defined in task.h
 * by FreeRTOS.
 */
typedef TaskHandle_t TASK_Hdl_t;

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
 * @brief Function reads the task configuration table
 * @param pTbl_ reference to the task configuration table
 * @return	error code, ERR_PARAM_ADDRESS, if address of input is invalid;
 * 						ERR_PARAM_DATA, if table data is invalid;
 * 						ERR_OK, if everything is OK
 */
EXTERNAL_ StdRtn_t TASK_Read_TaskCfgTbl(TASK_Cfg_t *pTbl_);

/**
 * @brief Function reads the task handle of the application task
 * @param pHdl_ reference to the task handle
 * @return	error code, ERR_PARAM_ADDRESS, if address of input is invalid;
* 						ERR_PARAM_DATA, if table data is invalid;
 * 						ERR_OK, if everything is OK
 */
EXTERNAL_ StdRtn_t TASK_Read_ApplTaskHdl(TASK_Hdl_t *pHdl_);

/**
 * @brief Function reads the task handle of the debug task
 * @param pHdl_ reference to the task handle
 * @return	error code, ERR_PARAM_ADDRESS, if address of input is invalid;
 * 						ERR_PARAM_DATA, if table data is invalid;
* 						ERR_OK, if everything is OK
 */
EXTERNAL_ StdRtn_t TASK_Read_DbgTaskHdl(TASK_Hdl_t *pHdl_);

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TASK_TYPES_H_ */
