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
typedef int32_t MTX_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ StdRtn_t MTX_Add(uint8_t sizeRows1_, uint8_t sizeCols1_, const MTX_t smd1_[sizeRows1_][sizeCols1_], uint8_t sizeRows2_, uint8_t sizeCols2_, const MTX_t smd2_[sizeRows2_][sizeCols2_], MTX_t sum_[sizeRows1_][sizeCols2_]);
EXTERNAL_ StdRtn_t MTX_Sub(uint8_t sizeRows1_, uint8_t sizeCols1_, const MTX_t min_[sizeRows1_][sizeCols1_], uint8_t sizeRows2_, uint8_t sizeCols2_, const MTX_t sub_[sizeRows2_][sizeCols2_], MTX_t diff_[sizeRows1_][sizeCols2_]);
EXTERNAL_ StdRtn_t MTX_Mult(uint8_t sizeRows1_, uint8_t sizeCols1_, const MTX_t fac1_[sizeRows1_][sizeCols1_], uint8_t sizeRows2_, uint8_t sizeCols2_, const MTX_t fac2_[sizeRows2_][sizeCols2_], MTX_t prod_[sizeRows1_][sizeCols2_]);
EXTERNAL_ StdRtn_t MTX_Trns(uint8_t  sizeRows1_, uint8_t sizeCols1_, const MTX_t trns_[sizeRows1_][sizeCols1_], uint8_t sizeRows2_, uint8_t sizeCols2_, const MTX_t divs_[sizeRows2_][sizeCols2_], MTX_t trnsp_[sizeRows1_][sizeCols1_]);
/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MTX_API_H_ */
