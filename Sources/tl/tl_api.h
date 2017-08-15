/***********************************************************************************************//**
 * @file		tl.h
 * @ingroup		tl
 * @brief 		tracking loop filter
 *
 * @author 		S. Helling, stu112498@tf.uni-kiel.de
 * @date		11.08.17
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef TL_API_H_
#define TL_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"


#ifdef MASTER_TL_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup TL
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
StdRtn_t TL_Read_CurLftPos(int32_t* result_);
StdRtn_t TL_Read_CurRghtPos(int32_t* result_);
int32_t  TL_Get_Speed(bool isLeft_);
/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MAF_API_H_ */
