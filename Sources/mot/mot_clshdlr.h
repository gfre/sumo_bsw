/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	29.03.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file motor_clshdl.h
 * 
 *==================================================================================================
 */


#ifndef MOT_CLSHDLR_H_
#define MOT_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


#ifdef MASTER_motor_clshdlr_C_
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
EXTERNAL_ uint8_t MOT_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !MOT_CLSHDLR_H_ */
