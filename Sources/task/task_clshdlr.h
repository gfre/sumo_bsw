/***********************************************************************************************//**
 * @file		task_clshdlr.h
 * @ingroup		task
 * @brief		Interface for the command line shell handler of the SWC @a Task
 *
 * This header files provides the interface from the SWC @ref task to the SWC @ref sh.
 * It introduces application specific commands for requests of status information
 * via command line shell (@b CLS).
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef TASK_CLSHDLR_H_
#define TASK_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_task_clshdlr_C_
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
 * @brief Parses a command
 * @param cmd_ Command string to be parsed
 * @param handled_ Sets this variable to TRUE if command was handled
 * @param io_ I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
EXTERNAL_ uint8 TASK_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TASK_CLSHDLR_H_ */
