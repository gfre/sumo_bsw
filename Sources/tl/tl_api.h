/***********************************************************************************************//**
 * @file		tl.h
 * @ingroup		tl
 * @brief 		Tracking loop filter
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date	11.08.17
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
typedef struct TL_vReadVal_s{
	int32_t val;
	const uint8_t idx;
}TL_vReadVal_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 *
 * @param pVal_
 * @param idx_
 * @return
 */
EXTERNAL_ StdRtn_t TL_Read_i32FltrdVal(int32_t* pVal_, uint8_t idx_);

/**
 *
 * @param pVal_
 * @param idx_
 * @return
 */
EXTERNAL_ StdRtn_t TL_Read_i16dFltrdValdt(int16_t* pVal_, uint8_t idx_);

/**
 *
 * @param pVal
 * @return
 */
EXTERNAL_ StdRtn_t TL_Read_vFltrdVal(void* pVal);

/**
 *
 * @param pVal
 * @return
 */
EXTERNAL_ StdRtn_t TL_Read_vdFltrdValdt(void* pVal);


/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MAF_API_H_ */
