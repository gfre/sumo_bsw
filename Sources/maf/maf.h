/***********************************************************************************************//**
 * @file		maf.h
 * @ingroup		maf
 * @brief 		Interface of the SWC @a Moving Average Filter for initialisation- and runtime-calls.
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

#ifndef MAF_H_
#define MAF_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "platform.h"


#ifdef MASTER_maf_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup maf
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the SWC @ref maf
 */
#define MAF_FILTER_STRING ("moving average filter")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
void MAF_Init(void);
void MAF_Main(void);
void MAF_Deinit(void);
void MAF_UpdateRingBuffer(int32_t leftVal, int32_t rightVal_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MAF_H_ */
