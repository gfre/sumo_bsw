/***********************************************************************************************//**
 * @file		mtx_extend.c
 * @ingroup		mtx
 * @brief 		This module implements extensions to the libfixmatrix library
 *
 *	This module implements QL-decomposition and UD-Decomposition aswell as a function that can
 *	append a matrix to another. QL-decomposition is similar to QR-decomposition with the diff-
 *	erence, that it returns a unit lower triangular matrix L and a orthonogonal not-normalized
 *	base vector matrix Q. UD-decomposition produces a a unit upper triangular matrix U and a
 *	diagonal matrix D for a symmetric, quadratic matrix P such that P=UDU'.
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.03.2018
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_mtx_extend_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "fixmatrix.h"
#include "fixarray.h"
#include "mtx_api.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void MTX_SubtractProjection(fix16_t *v, const fix16_t *u, fix16_t dot, int n, uint8_t *errors);


/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void MTX_SubtractProjection(fix16_t *v, const fix16_t *u, fix16_t dot, int n, uint8_t *errors)
{
    while (n--)
    {
        // For unit vector u, u[i] <= 1
        // Therefore this multiplication cannot overflow
        fix16_t product = fix16_mul(dot, *u);

        // Overflow here is rare, but possible.
        fix16_t diff = fix16_sub(*v, product);

        if (diff == fix16_overflow)
            *errors |= FIXMATRIX_OVERFLOW;

        *v = diff;

        v += FIXMATRIX_MAX_SIZE;
        u += FIXMATRIX_MAX_SIZE;
    }
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void MTX_AppendMatrix(MTX_t *dest_, const MTX_t *a_, const MTX_t *b_, const uint8_t posRow_, const uint8_t posColumn_)
{
	int row, column;

	if( (posRow_ < a_->rows) || (posColumn_ < a_->columns))
		dest_->errors |= FIXMATRIX_USEERR;

	if(dest_ != a_)
		*dest_ = *a_;

	if(b_->rows < a_->rows && (posRow_ < a_->rows))
		dest_->rows = a_->rows;
	else
		dest_->rows    = posRow_ + b_->rows - 1;
	if(b_->columns < a_->columns && posColumn_ < a_->columns)
		dest_->columns = a_->columns;
	else
		dest_->columns = posColumn_ + b_->columns - 1;

	dest_->errors  = (a_->errors | b_->errors);

	if( (dest_->rows > FIXMATRIX_MAX_SIZE) || (dest_->columns > FIXMATRIX_MAX_SIZE) )
		dest_->errors |= FIXMATRIX_USEERR;

	for(row = (posRow_-1); row < dest_->rows; row++)
	{
		for(column = (posColumn_-1); column < dest_->columns; column++)
		{
			dest_->data[row][column] = b_->data[row-(posRow_-1)][column-(posColumn_-1)];
		}
	}
}

void MTX_QlDecomposition(MTX_t *q_, MTX_t *l_, const MTX_t *mtx_, const uint8_t reOrthCnt_)
{
    uint8_t i = 0u, j = 0u, reorth = 0u;
    fix16_t dotaiqip1 = 0, dotqip1qip1 = 0;
    uint8_t stride = FIXMATRIX_MAX_SIZE;
    uint8_t n = mtx_->rows;

    // We start with q_ = mtx_
    if (q_ != mtx_)
    {
        *q_ = *mtx_;
    }

    // l_ is initialized to have diagonal elements set to 1.
    l_->columns = mtx_->columns;
    l_->rows    = mtx_->columns;
    l_->errors  = 0;
    mf16_fill_diagonal(l_, fix16_one);

    // Now do the actual Gram-Schmidt for the rows.
    for (j = 1; j < q_->columns; j++)
    {
        for (reorth = 0; reorth <= reOrthCnt_; reorth++)
        {
            for (i = 0; i < j; i++)
            {
                fix16_t *ai   = &q_->data[0][(q_->columns-1)-j];
                fix16_t *qip1 = &q_->data[0][(q_->columns-1)-i];

                dotaiqip1   = fa16_dot(ai, stride, qip1, stride, n);
                dotqip1qip1 = fa16_dot(qip1, stride, qip1, stride, n);
                l_->data[(q_->columns-1)-i][(q_->columns-1)-j] = fix16_div(dotaiqip1, dotqip1qip1);
                MTX_SubtractProjection(ai, qip1, l_->data[(q_->columns-1)-i][(q_->columns-1)-j], n, &q_->errors);

                if ( (dotaiqip1 == fix16_overflow) || (dotqip1qip1 == fix16_overflow))
                    q_->errors |= FIXMATRIX_OVERFLOW;
            }
        }
    }

    l_->errors = q_->errors;
}


void MTX_UdDecomposition(MTX_t *u_, MTX_t *d_, const MTX_t *mtx_)
{
	int8_t i = 0, j = 0, k = 0;
	fix16_t sigma = 0;
	fix16_t tmp   = 0;
	d_->errors = mtx_->errors;
	u_->errors = mtx_->errors;

	if (mtx_->rows != mtx_->columns)
	{
			u_->errors |= FIXMATRIX_DIMERR;
			d_->errors |= FIXMATRIX_DIMERR;
	}
	u_->rows    = mtx_->rows;
	u_->columns = mtx_->rows;
	d_->rows    = mtx_->rows;
	d_->columns = mtx_->rows;

	for (j = (mtx_->rows-1); j >= 0; j--)
	{
		for (i = j; i >= 0; i--)
		{
				sigma = mtx_->data[i][j];
				for(k = (j+1); (k < mtx_->rows); k++)
				{
					tmp = fix16_mul(u_->data[i][k], d_->data[k][k]);
					tmp = fix16_mul(tmp,           u_->data[j][k]);
					if(fix16_overflow == tmp)
					{
						u_->errors |= FIXMATRIX_OVERFLOW;
						d_->errors |= FIXMATRIX_OVERFLOW;
						return;
					}
					sigma = fix16_sub(sigma, tmp);
					if(fix16_overflow == sigma)
					{
						u_->errors |= FIXMATRIX_OVERFLOW;
						d_->errors |= FIXMATRIX_OVERFLOW;
						return;
					}
				}
				if(i == j)
				{
					d_->data[j][j] = sigma;
					u_->data[j][j] = fix16_from_int(1);
				}
				else
				{
					u_->data[i][j] = fix16_div(sigma, d_->data[j][j]);
					u_->data[j][i] = 0;
					if(fix16_overflow == u_->data[i][j])
					{
						u_->errors |= FIXMATRIX_OVERFLOW;
						d_->errors |= FIXMATRIX_OVERFLOW;
						return;
					}
				}
		}
	}
}



#ifdef MASTER_mtx_extend_C_
#undef MASTER_mtx_extend_C_
#endif /* !MASTER_mtx_extend_C_ */
