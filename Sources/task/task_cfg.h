/***************************************************************************************************
 * @brief 	This module configures all tasks
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file task_cfg.h
 * 
 *==================================================================================================
 */


#ifndef TASK_CFG_H_
#define TASK_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "task_api.h"



#ifdef MASTER_task_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#define APPL_TASK_STRING  ("APPL")
#define COMM_TASK_STRING  ("COMM")
#define DBG_TASK_STRING   ("DEBUG")
#define DRV_TASK_STRING   ("DRIVE")



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/*!
 * @brief Function returns the entire configuration of all tasks
 * @return pointer to the entire task configuration
 */
EXTERNAL_ const TASK_Cfg_t *TASK_Get_TasksCfg(void);

/*!
 * @brief Function returns the configuration of the application task
 * @return pointer to the configuration of the application task
 */
EXTERNAL_ const TASK_CfgItm_t *TASK_Get_ApplTaskCfg(void);

/*!
 * @brief Function returns the configuration of the debug task
 * @return pointer to the configuration of the debug task
 */
EXTERNAL_ const TASK_CfgItm_t *TASK_Get_DbgTaskCfg(void);



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !TASK_CFG_H_ */
