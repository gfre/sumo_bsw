/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	23.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file nvm_cls.h
 * 
 *==================================================================================================
 */


#ifndef NVM_CLSHDLR_H_
#define NVM_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


#ifdef MASTER_nvm_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0xFFu) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/* typedef unit8 TmplType_t; */



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ uint8_t NVM_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !NVM_CLSHDLR_H_ */
