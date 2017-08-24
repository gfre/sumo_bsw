/***********************************************************************************************//**
 * @file		mtx_api.h
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	<I>. <Surname>, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	17.08.2017
 *
 * @copyright @<LGPLx_x>
 *
 ***************************************************************************************************/

#ifndef MTX_API_H_
#define MTX_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Acon_Types.h"
#include "Platform.h"



#ifdef MASTER_mtx_api_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup <group label>
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct MTX_s
{
	int32_t *pData;
	uint8_t  NumRows;
	uint8_t	 NumCols;
}MTX_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ StdRtn_t MTX_Add(const MTX_t *smd1_, const MTX_t *smd2_, MTX_t *sum_);
EXTERNAL_ StdRtn_t MTX_Sub(const MTX_t *min_, const MTX_t *sub_, MTX_t *diff_);
EXTERNAL_ StdRtn_t MTX_Mult(const MTX_t *fac1_, const MTX_t *fac2_, MTX_t *prod_);
EXTERNAL_ StdRtn_t MTX_Trns(const MTX_t *trns_, const MTX_t *divs_, MTX_t *trnsp_);
/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MTX_API_H_ */