/***********************************************************************************************//**
 * @file		npid_clshdlr.h
 * @ingroup		pid
 * @brief		Interface for the command line shell handler of the SWC @a PID
 *
 * This header files provides the interface from the SWC @ref pid to the SWC @ref sh.
 * It introduces application specific commands for requests of status information,
 * changing PID controller parameters, or restoring them from NVM via command line shell (@b CLS).
 * The changed parameters are immediately saved to the @ref nvm.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	28.02.2017
 *  
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef PID_CLSHDLR_H_
#define PID_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_pid_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup pid
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Shell command line parser.
 * @param cmd Pointer to command string
 * @param handled If command is handled by the parser
 * @param io Std I/O handler of shell
 */
EXTERNAL_ uint8_t PID_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !PID_CLSHDLR_H_ */
