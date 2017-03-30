/***************************************************************************************************
 * @brief 	Command line shell handler of the software component of the PID controllers.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	28.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module handles the interface between the software component of the PID controllers
 * and the command line shell CLS.
 * 
 *==================================================================================================
 */


#ifndef PID_CLSHDLR_H_
#define PID_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


#ifdef MASTER_pid_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Shell command line parser.
 * @param[in] cmd Pointer to command string
 * @param[out] handled If command is handled by the parser
 * @param[in] io Std I/O handler of shell
 */
EXTERNAL_ uint8_t PID_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !PID_CLSHDLR_H_ */
