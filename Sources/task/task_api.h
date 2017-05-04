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
#include "projdefs.h"
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
 * @brief
 */
typedef TaskFunction_t TaskFctHdl_t;

/**
 * @brief
 */
typedef void * TaskHdl_t;

/**
 * @typedef TASK_SuspType_t
 * @brief
 *
 * @enum TASK_SuspType_e
 * @brief
 */
typedef enum TASK_SuspType_e
{
	 TASK_SUSP_NEVER = 0x00		  	/**< task will be never suspended */
	,TASK_SUSP_DEFAULT				/**< task is suspended at default */
}TASK_SuspType_t;

/**
 * @typedef TASK_CfgItm_t
 * @brief
 *
 * @struct TASK_CfgItm_s
 * @brief
 */
typedef struct TASK_CfgItm_s
{
	TaskFctHdl_t taskFctHdl;		/**< */
	const char_t * const taskName;	/**< */
	const uint16 stackDepth;		/**< */
	void * const pvParameters;		/**< */
	uint32 taskPriority;			/**< */
	TaskHdl_t taskHdl;				/**< */
	TASK_SuspType_t suspTask;		/**< */
}TASK_CfgItm_t;

/**
 * @typedef TASK_Cfg_t
 * @brief
 *
 * @struct TASK_Cfg_s
 * @brief
 */
typedef struct TASK_Cfg_s
{
	TASK_CfgItm_t *tasks;			/**< */
	const uint8 numTasks;			/**< */
}TASK_Cfg_t;

/**
 * @brief
 */
typedef void (TASK_MainFct_t)(void);

/**
 * @brief
 */
typedef void (TASK_InitFct_t)(void);

/**
 * @typedef TASK_SwcCfg_t
 * @brief
 *
 * @struct TASK_SwcCfg_s
 * @brief
 */
typedef struct TASK_SwcCfg_s
{
	const char_t * const swcName;	/**< */
	TASK_MainFct_t * const mainFct;	/**< */
	TASK_InitFct_t * const initFct;	/**< */
}TASK_SwcCfg_t;

/**
 * @typedef TASK_PerdTaskFctPar_t
 * @brief
 *
 * @struct TASK_PerdTaskFctPar_s
 * @brief
 */
typedef struct TASK_PerdTaskFctPar_s
{
	const uint8 taskPeriod;			/**< */
	const TASK_SwcCfg_t *swcCfg;	/**< */
	const uint8 numSwc;				/**< */
}TASK_PerdTaskFctPar_t;

/**
 * @typedef TASK_NonPerdTaskFctPar_t
 * @brief
 *
 * @struct TASK_NonPerdTaskFctPar_s
 * @brief
 */
typedef struct TASK_NonPerdTaskFctPar_s
{
	const uint8 taskDelay;			/**< */
	const TASK_SwcCfg_t *swcCfg;	/**< */
	const uint8 numSwc;				/**< */
}TASK_NonPerdTaskFctPar_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief
 * @param pvParameters
 */
EXTERNAL_ void TASK_PerdTaskFct(void *pvParameters);

/**
 * @brief
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
