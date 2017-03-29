/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	27.03.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file ind.h
 * 
 *==================================================================================================
 */


#ifndef IND_H_
#define IND_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "FRTOS1.h"
#include "ACon_Types.h"

#ifdef MASTER_ind_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0xFFu) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/




/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ void IND_Init(void);

EXTERNAL_ void IND_Main(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !IND_H_ */
