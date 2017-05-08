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



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Function returns the entire configuration of all tasks
 * @return pointer to the entire task configuration
 */
EXTERNAL_ const TASK_Cfg_t *TASK_Get_TasksCfg(void);

/**
 * @brief Function returns the configuration of the application task
 * @return pointer to the configuration of the application task
 */
EXTERNAL_ const TASK_CfgItm_t *TASK_Get_ApplTaskCfg(void);

/**
 * @brief Function returns the configuration of the debug task
 * @return pointer to the configuration of the debug task
 */
EXTERNAL_ const TASK_CfgItm_t *TASK_Get_DbgTaskCfg(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TASK_CFG_H_ */
