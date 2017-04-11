/***************************************************************************************************
 * @brief 	This module handles the interface to the command line shell CLS
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, University Kiel
 * @date 	09.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file sh_clshdlr.h
 * 
 *==================================================================================================
 */


#ifndef SH_CLSHDLR_H_
#define SH_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


#ifdef MASTER_sh_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Sends a error string to the shell/console stdout
 * @param *cmd_ pointer to command
 * @param *handled_ pointer to flag which returns TRUE if cmd_ was handled_, otherwise FALSE (CBR).
 * @param *io_ pointer to command line shell standard IO type
 * @return always ERR_OK
 */
EXTERNAL_ uint8_t SH_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !SH_CLSHDLR_H_ */
