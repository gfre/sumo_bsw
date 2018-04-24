/***********************************************************************************************//**
 * @file		tl.h
 * @ingroup		tl
 * @brief 		API of the SWC @a tl
 *
 * This API provides a BSW-internal interface of the SWC @ref tl. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author 	S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct TL_vReadVal_s{
	int32_t val;
	const uint8_t idx;
}TL_vReadVal_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Returns the estimated value \f$\hat{x}_1\f$ for the measured state \f$ x_1\f$
 *        as an int32_t
 * @param pVal_ Pointer to the value
 * @param idx_ ID that corresponds to an item in the @ref TL_ItmTbl_t table
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_INDEX if idx_ is not in the item table
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TL_Read_i32FltrdVal(int32_t* pVal_, uint8_t idx_);

/**
 * @brief Returns the estimated value \f$\hat{x}_2\f$ for the state \f$ x_2\f$
 *        as an int16_t
 * @param pVal_ Pointer to the value
 * @param idx_ ID that corresponds to an item in the @ref TL_ItmTbl_t table
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_INDEX if idx_ is not in the item table
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TL_Read_i16dFltrdValdt(int16_t* pVal_, uint8_t idx_);

/**
 *
 * @param pVal_
 */
EXTERNAL_ StdRtn_t TL_Read_vFltrdVal(void* pVal_);

/**
 *
 * @param pVal_
 */
EXTERNAL_ StdRtn_t TL_Read_vdFltrdValdt(void* pVal_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TL_API_H_ */
