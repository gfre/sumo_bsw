/***********************************************************************************************//**
 * @file		mtx_api.h
 * @ingroup		mtx
 * @brief 		API for the SWC @a mtx
 *
 * This API provides a BSW-internal interface of the SWC @ref mtx. It is supposed to be
 * available to all other Basic Software Components.
 *
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author  S. Helling,		  stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.03.2018
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
 * @addtogroup mtx
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * @brief Produces A = ones(rows(A), cols(A)) * value
 * @param[in] val_ = value
 * @param[in,out] dest_ = A
 */
#define MTX_Fill(dest_, val_) (mf16_fill(dest_, val_))

/**
 * @brief Produces A = diag{value}
 * @param[in] val_ = value
 * @param[in,out] dest_ = A
 */
#define MTX_FillDiagonal(dest_, val_) (mf16_fill_diagonal(dest_, val_))

/**
 * @brief Produces A = B * C
 * @param[in,out] dest_ = A
 * @param[in] fac1_ = B
 * @param[in] fac2_ = C
 * @return dest_->errors = FIXMATRIX_DIMERR if dimensions don't agree <br>
 * 		   dest_->errors = FIXMATRIX_OVERFLOW if any overflow occurred
 */
#define MTX_Mult(dest_, fac1_, fac2_) (mf16_mul(dest_, fac1_, fac2_))

/**
 * @brief Produces A = B' * C
 * @param[in,out] dest_ = A
 * @param[in] fac1_ = B
 * @param[in] fac2_ = C
 * @return dest_->errors = FIXMATRIX_DIMERR if dimensions don't agree <br>
 * 		   dest_->errors FIXMATRIX_OVERFLOW if any overflow occurred
 */
#define MTX_MultAt(dest_, fac1_, fac2_) (mf16_mul_at(dest_, fac1_, fac2_))

/**
 * @brief Produces A = B * C'
 * @param[in,out] dest_ = A
 * @param[in] fac1_ = B
 * @param[in] fac2_ = C
 * @return dest_->errors = FIXMATRIX_DIMERR if dimensions don't agree <br>
 * 		   dest_->errors = FIXMATRIX_OVERFLOW if any overflow occurred
 */
#define MTX_MultBt(dest_, fac1_, fac2_) (mf16_mul_bt(dest_, fac1_, fac2_))

/**
 * @brief Produces A = B + C.
 * @param[in,out] dest_ = A
 * @param[in] sum1_ = B
 * @param[in] sum2_ = C
 * @remark sum1_ and sum2_ can alias with dest_, respectively
 * @return dest_->errors = FIXMATRIX_DIMERR if dimensions don't agree <br>
 * 		   dest_->errors = FIXMATRIX_OVERFLOW if any overflow occurred
 */
#define MTX_Add(dest_, sum1_, sum2_) (mf16_add(dest_, sum1_, sum2_))

/**
 * @brief Produces A = B - C.
 * @param[in,out] dest_ = A
 * @param[in] min_ = B
 * @param[in] sub_ = C
 * @remark min_ and sub_ can alias with dest_, respectively
 * @return dest_->errors = FIXMATRIX_DIMERR if dimensions don't agree <br>
 * 		   dest_->errors = FIXMATRIX_OVERFLOW if any overflow occurred
 */
#define MTX_Sub(dest_, min_, sub_) (mf16_sub(dest_, min_, sub_))

/**
 * @brief Calculates the transpose A = B' and copies errors.
 * @param[in,out] dest_ = A
 * @param[in] mtx_ = B
 * @remark dest_ and mtx_ can alias
 */
#define MTX_Transpose(dest_, mtx_) (mf16_transpose(dest_, mtx_))

/**
 * @brief Produces A = B * s with s being a scalar
 * @param[in,out] dest_ = A
 * @param[in] mtx_ = B
 * @param[in] val_ = s
 * @return dest_->errors = FIXMATRIX_OVERFLOW if any overflow occurred
 */
#define MTX_MultScalar(dest_, mtx_, val_) (mf16_mul_s(dest_, mtx_, val_))

/**
 * @brief Produces A = B / s with s being a scalar
 * @param[in,out] dest_= A
 * @param[in] mtx_ = B
 * @param[in] val_ = s
 * @return dest_->errors = FIXMATRIX_OVERFLOW if any overflow occurred
 */
#define MTX_DivScalar(dest_, mtx_, val_) (mf16_div_s(dest_, mtx_, val_))

/**
 * @brief Decomposes a matrix A into a new set of orthonormal base vectors
 * 		  in Q and a upper  triangular matrix R such that A = Q * R.
 * @param[in] mtx_ = A
 * @param[in,out] q_ = Q
 * @param[in,out] r_ = R <br>
 * @param[in] reOrthCnt_ reorthogonalizes the matrix which produces more accurate results.
 * @remark q_ and mtx_ and r_ and mtx_ may alias, respectively
 * @return q_->errors = r_->errors = FIXMATRIX_OVERFLOW if any overflow occurred <br>
 * 		   q_->errors = r_->errors = FIXMATRIX_SINGULAR if a division by 0 occurred
 *
 */
#define MTX_QrDecomposition(q_, r_, mtx_, reOrthCnt_) (mf16_qr_decomposition(q_, r_, mtx_, reOrthCnt_))

/**
 * @brief Solves a system of linear equations Ax = b by using QR-factors of A
 * 		  => (QR)x=b.
 * @param[in] mtx_ = b
 * @param[in,out] dest_ = x
 * @param[in] q_ = Q
 * @param[in] r_ = R
 * @remark dest_ can alias with mtx_ or q_ but not with r_
 * @return dest_->errors = FIXMATRIX_OVERFLOW if any overflow occurred <br>
 * 		   dest_->errors = FIXMATRIX_USEERR if dimensions don't agree or r_=dest_<br>
 * 		   dest_->errors = FIXMATRIX_SINGULAR if a division by 0 occurred
 */
#define MTX_Solve(dest_, q_, r_, mtx_) (mf16_solve(dest_, q_, r_, mtx_))

/**
 * @brief Decomposes a symmetric, positive-definite matrix A such that A = L * L',
 * 		  L is a lower triangular matrix.
 * @param[in] mtx_ = A
 * @param[in,out] dest_ = L
 * @remark dest_ and mtx_ can alias
 * @return dest_->errors = FIXMATRIX_DIMERR if A is not square <br>
 * 		   dest_->errors = FIXMATRIX_OVERFLOW if any overflow occurred <br>
 * 		   dest_->errors = FIXMATRIX_NEGATIVE if error in square root occurred
 */
#define MTX_Cholesky(dest_, mtx_) (mf16_cholesky(dest_, mtx_))

/**
 * @brief Inversion of a matrix A through its decomposition A = L * L'.
 * 		  L may be obtained with MTX_Cholesky.
 * @param[in] mtx_ = L
 * @param[out] dest_ = A^{-1}
 * @remark dest_ and mtx_ can alias.
 */
#define MTX_InvertLowerTri(dest_, mtx_) (mf16_invert_lt(dest_, mtx_))



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @brief The fix size of this type can be changed in fixmatrix.h through FIXMATRIX_MAX_SIZE
 */
typedef mf16 MTX_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Appends matrix B to matrix A at a specified position
 * @param[in,out] dest_ = augmented matrix
 * @param[in] a_ = A
 * @param[in] b_ = B
 * @param[in] posRow_ the row where B should be put
 * @param[in] posColumn_ the column where B should be put
 * @return dest_->errors = FIXMATRIX_USEERR if matrix exceeds FIXMATRIX_MAX_SIZE
 * 		   or if B would overwrite A
 */
EXTERNAL_ void MTX_AppendMatrix(MTX_t *dest_, const MTX_t *a_, const MTX_t *b_, const uint8_t posRow_, const uint8_t posColumn_);

/**
 * @brief Decomposes a matrix A into a new set of orthogonal base vectors in Q and a unit lower
 * 		  triangular matrix L such that A = Q * L
 * @param[in] mtx_ = A
 * @param[in,out] q_ = Q
 * @param[in,out] l_ = L
 * @param[in] reOrthCnt_ reorthogonalizes the matrix which produces more accurate results
 * @return q_->errors = l_->errors = FIXMATRIX_OVERFLOW if any overflow occurred
 */
EXTERNAL_ void MTX_QlDecomposition(MTX_t *q_, MTX_t *l_, const MTX_t *mtx_, const uint8_t reOrthCnt_);

/**
 * @brief Decomposes a symmetric, positve-definite matrix A into a unit upper triangular matrix U and a
 * 		  diagonal matrix D such that A = U * D * U'
 * @param[in,out] u_ = U
 * @param[in,out] d_ = D
 * @param[in] mtx_ = A
 * @return u_->errors = d_->errors = FIXMATRIX_OVERFLOW if any overflow occurred
 */
EXTERNAL_ void MTX_UdDecomposition(MTX_t *u_, MTX_t *d_, const MTX_t *mtx_);

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MTX_API_H_ */
