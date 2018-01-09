/***********************************************************************************************//**
 * @file		mtx_api.h
 * @ingroup		mtx
 * @brief 		implements user interface for fixed point matrix operations including addition,
 * 				subtraction and multiplication of different Q-formatted matrices/vectors aswell
 * 				as UD-Decomposition for symmetric matrices.
 *
 * This module works with matrices of the provided type 'MTX_t'. It consists of a pointer to a
 * int32_t matrix that is initialized with e.g. 'int32_t arr[rows][cols];',
 * which declares a 'rows' by 'cols' matrix. Note that it cannot be initialized if 'rows' and 'cols'
 * are not defined somewhere in the workspace.
 * The MTX_t struct is initialized with e.g. 'MTX_t mat = {&arr[0], rows, cols, m, n};', where
 * 'rows' and 'cols' must be equal to the values in the corresponding int32_t array it points to.
 * This struct also carries information of the Q-format type (Qm.n) of the fixed point matrix.
 * In this, 'm' denotes the number of integer bits and 'n' specifies the number of fractional bits
 * in the matrix.
 * Each of the basic mathematical operation functions can receive different-formatted types of
 * matrices and adjust the Q-format of the resulting matrix/vector accordingly. Note that when
 * initializing a matrix, 'm' should represent the range in which the integer part of the desired
 * matrix is, e.g. if it contains integer values below and including 256, 'm' should be initialized
 * with '8' (2^8 = 256). Furthermore, note that these values are not constant and can change
 * depending on the operations and formats of other matrices being used with this matrix.
 *
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author  S. Helling,		  stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	17.08.2017
 *
 * @copyright @<LGPL2_1>
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
#define MTX_ij(mtx_, i_, j_) ( mtx_->pData[i_*mtx_->NumCols + j_] )
#define MTXLOC_ij(mtx_, i_, j_) ( mtx_.pData[i_*mtx_.NumCols + j_] )





/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct MTX_s
{
	int32_t *pData;
	uint8_t  NumRows;
	uint8_t	 NumCols;
	uint8_t	 NumFractionalBits; /* 'scaling' */
}MTX_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief  Adds two matrices which can be of different Q-format.
 * @param  smd1_ is the first and smd2_ the second summand.
 * @return Sum of the two summands
 */
EXTERNAL_ StdRtn_t MTX_Add(MTX_t *smd1_, MTX_t *smd2_, MTX_t *sum_);

/**
 * @brief  Subtracts two matrices which can be of different Q-format.
 * @param  min_ is the minuend and sub_ subtrahend.
 * @return difference between minuend and subtrahend
 */
EXTERNAL_ StdRtn_t MTX_Sub(MTX_t *min_,  MTX_t *sub_, MTX_t *diff_);

/**
 * @brief  Multiplicates two matrices which can be of different Q-format.
 * @param  fac1_ is the first and fac2_ the second factor.
 * @return Product of the two matrices
 */
EXTERNAL_ StdRtn_t MTX_Mult(MTX_t *fac1_, MTX_t *fac2_, MTX_t *prod_);

/**
 * @brief Scales up a matrix by a power of '2'.
 * @param mtx_ is the matrix which contents are leftshifted by nScale_ bits.
 * @return Scaled up matrix
 */
EXTERNAL_ StdRtn_t MTX_ShiftLeft(MTX_t *mtx_, const uint8_t nScale_);

/**
 * @brief Scales down a matrix by a power of '2'.
 * @param mtx_ is the matrix which contents are rightshifted by nScale_ bits.
 * @return Scaled down matrix
 */
EXTERNAL_ StdRtn_t MTX_ShiftRight(MTX_t *mtx_, const uint8_t nScale_);

/**
 * @brief  Transposes a matrix.
 * @param  mtx_ is the matrix that is to be transposed.
 * @return Transposed matrix mtxTrnspsd_
 */
EXTERNAL_ StdRtn_t MTX_Transpose(const MTX_t *mtx_, MTX_t *mtxTrnspsd_);

/**
 * @brief Fills up an entire matrix with values between 0 and 255.
 * @param mtx_ is the matrix which contents will be overwritten with val_.
 * @return Filled matrix
 */
EXTERNAL_ StdRtn_t MTX_Fill(MTX_t *mtx_, const uint8_t val_, const uint8_t nFractionalBits_);

/**
 * @brief Fills up the diagonal of a matrix with values between 0 and 255.
 * @param mtx_ is the matrix which diagonal will be overwritten with val_.
 * @return Diagonal matrix
 */
EXTERNAL_ StdRtn_t MTX_FillDiagonal(MTX_t *mtx_, const uint8_t val_, const uint8_t nFractionalBits_);

/**
 * @brief Copies one matrix into another.
 * @param mtx1_ is the matrix which shall be copied.
 * @return mtx2_ is the same matrix as  mtx1_
 */
EXTERNAL_ StdRtn_t MTX_Copy(MTX_t *mtx1_, MTX_t *mtx2_);

/**
 * @brief Decomposes a symmetric matrix M into a (scaled) upper triangular matrix U
 * 		  and a diagonal matrix D such that M = UDU^T.
 * @param mtx_ is the matrix that will be decomposed, nScaleU_ the scale U will be in (Q1.nScaleU_).
 * @return Scaled upper triangular matrix U and diagonal matrix D
 */
EXTERNAL_ StdRtn_t MTX_UdDecomposition(const MTX_t *mtx_, MTX_t *mtxu_, MTX_t *mtxd_, const uint8_t nScaleU_);

EXTERNAL_ StdRtn_t MTX_OverFlowMult(const int32_t fac1_, const int32_t fac2_, int32_t *prod_, uint8_t *nRightShift);

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MTX_API_H_ */
