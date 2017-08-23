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
/**
 * String identification of the SWC @ref tl
 */
#define TL_SWC_STRING ("tracking loop")


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct TL_vReadVal_s{
	int32_t val;
	const uint8_t idx;
}TL_vReadVal_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 *
 * @param sig_
 * @param idx_
 * @return
 */
StdRtn_t TL_Read_i32FltrdVal(int32_t* sig_, uint8_t idx_);

/**
 *
 * @param sig_
 * @param idx_
 * @return
 */
StdRtn_t TL_Read_i32dFltrdValdt(int32_t* sig_, uint8_t idx_);

/**
 *
 * @param pSig_
 * @return
 */
StdRtn_t TL_Read_vFltrdVal(void* pSig_);

/**
 *
 * @param pSig_
 * @return
 */
StdRtn_t TL_Read_vdFltrdValdt(void* pSig_);


/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MAF_API_H_ */
