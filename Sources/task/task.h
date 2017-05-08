/***********************************************************************************************//**
 * @file		task.h
 * @ingroup		task
 * @brief 		Interface of the SWC @a Task for the initialisation call.
 *
 * This header file provides the internal interface between the SWC @ref task and the real-time
 * operating system FreeRTOS. The initialisation of the FreeRTOS task is called in the main()
 * routine of the C-project.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef TASK_H_
#define TASK_H_

/*======================================= >> #INCLUDES << ========================================*/



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



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Creates the tasks and initialises the software components run by the created tasks
 */
EXTERNAL_ void TASK_Init(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TASK_H_ */
