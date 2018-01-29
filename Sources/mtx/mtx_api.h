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
#include "fixmatrix.h"




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
#define MTX_Fill(dest_, val_) (mf16_fill(dest_, val_))
#define MTX_FillDiagonal(dest_, val_) (mf16_fill_diagonal(dest_, val_))
#define MTX_Mult(dest_, fac1_, fac2_) (mf16_mul(dest_, fac1_, fac2_))
#define MTX_MultAt(dest_, fac1T_, fac2_) (mf16_mul_at(dest_, fac1T_, fac2_))
#define MTX_MultBt(dest_, fac1_, fac2T_) (mf16_mul_bt(dest_, fac1_, fac2T_))
#define MTX_Add(dest_, sum1_, sum2_) (mf16_add(dest_, sum1_, sum2_))
#define MTX_Sub(dest_, min_, sub_) (mf16_sub(dest_, min_, sub_))
#define MTX_Transpose(dest_, mtx_) (mf16_transpose(dest_, mtx_))
#define MTX_MultScalar(dest_, mtx_, val_) (mf16_mul_s(dest_, mtx_, val_))
#define MTX_DivScalar(dest_, mtx_, val_) (mf16_div_s(dest_, mtx_, val_))
#define MTX_QrDecomposition(q_, r_, mtx_, reorthCnt_) (mf16_qr_decomposition(q_, r_, mtx_, reorthCnt_))
#define MTX_Solve(dest_, q_, r_, mtx_) (mf16_solve(dest_, q_, r_, mtx_))
#define MTX_Cholesky(dest_, mtx_) (mf16_cholesky(dest_, mtx_))
#define MTX_InvertLowerTri(dest_, mtx_) (mf16_invert_lt(dest_, mtx_))



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef mf16 MTX_t;
typedef fix16_t int32_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MTX_API_H_ */
