/***********************************************************************************************//**
 * @file		tl.h
 * @ingroup		tl
 * @brief 		Implements a tracking loop filter to estimate velocity
 *

 *
 * @author 	S. Helling, stu112498@uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.08.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef TL_H_
#define TL_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"
#include "Platform.h"

#ifdef MASTER_tl_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup tl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 *
 */
EXTERNAL_ void TL_Init(void);

/**
 *
 */
EXTERNAL_ void TL_Main(void);

/**
 *
 */
EXTERNAL_ void TL_DeInit(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TL_H_ */
