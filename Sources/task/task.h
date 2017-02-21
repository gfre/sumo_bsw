/***************************************************************************************************
 * @brief 	This module creates and runs all tasks
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file task.h
 * 
 *==================================================================================================
 */


#ifndef TASK_H_
#define TASK_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_task_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Run the software
 */
EXTERNAL_ void TASK_Run(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !TASK_H_ */
