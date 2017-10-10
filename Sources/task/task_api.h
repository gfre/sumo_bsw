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
