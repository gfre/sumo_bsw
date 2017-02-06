/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, University Kiel
 * @date 	03.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file sh_cfg.h
 * 
 *==================================================================================================
 */


#ifndef SH_CFG_H_
#define SH_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


#ifdef MASTER_sh_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/




/*=================================== >> TYPE DEFINITIONS << =====================================*/




/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
const CLS1_ParseCommandCallback *Get_CmdParserTbl();


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !SH_CFG_H_ */
