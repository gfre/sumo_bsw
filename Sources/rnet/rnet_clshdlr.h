/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	17.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file rnet_clshdlr.h
 * 
 *==================================================================================================
 */


#ifndef RNET_CLSHDLR_H_
#define RNET_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


#ifdef MASTER_rnet_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0xFFu) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/* typedef unit8 TmplType_t; */



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ uint8_t RNET_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !RNET_CLSHDLR_H_ */
