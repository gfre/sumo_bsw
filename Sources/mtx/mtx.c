/***********************************************************************************************//**
 * @file		mtx.c
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

#define MASTER_mtx_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "mtx.h"
#include "mtx_api.h"

/*======================================= >> #DEFINES << =========================================*/
#define MTX1(i_, j_) ( MTX_ij(mtx1_, i_, j_) )
#define MTX2(i_, j_) ( MTX_ij(mtx2_, i_, j_) )
#define MTXRes(i_, j_) ( MTX_ij(mtxRes_, i_, j_) )

#define SIGN(x_) ( (x_ > 0) ? (1) : ( (x_ < 0) ? (-1) : (0) ) )


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum MTX_Op_e
{
	 MTX_ADD = 0
	,MTX_SUB
	,MTX_MULT
	,MTX_SCALE_UP
	,MTX_SCALE_DOWN
	,MTX_TRNS
	,MTX_FILL
	,MTX_FILL_DIAG
	,MTX_COPY
	,MTX_CNT_OF_OPS
}MTX_Op_t;


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static inline void MtxOverFlowMult(const int32_t fac1_, const int32_t fac2_, int32_t *prod_, uint8_t *nRightShift_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static inline StdRtn_t MtxCalc(const MTX_t *mtx1_, const MTX_t *mtx2_, MTX_Op_t op_, MTX_t *mtxRes_, uint8_t nScale_)
{
	uint8_t i=0u, j=0u, k=0u;
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if ( NULL != mtxRes_ )
	{
		retVal = ERR_OK;
		for(i = 0; (i < mtx1_->NumRows) && (ERR_OK == retVal); i++)
		{
			for(j = 0; (j < mtx2_->NumCols) && (ERR_OK == retVal); j++)
			{
				switch(op_)
				{
				case MTX_ADD:
					if( ((mtx1_->NumRows == mtx2_->NumRows) && (mtx1_->NumCols == mtx2_->NumCols)) && ((mtxRes_->NumRows == mtx1_->NumRows) && (mtxRes_->NumCols == mtx1_->NumCols))  )  //dimensions must agree
					{
						MTXRes(i,j) = MTX1(i,j) + MTX2(i,j);
					}
					else
					{
						retVal = ERR_PARAM_SIZE;
					}
					break;
				case MTX_SUB:
					if( ((mtx1_->NumRows == mtx2_->NumRows) && (mtx1_->NumCols == mtx2_->NumCols)) && ((mtxRes_->NumRows == mtx1_->NumRows) && (mtxRes_->NumCols == mtx1_->NumCols))  )  //dimensions must agree
					{
						MTXRes(i,j) = MTX1(i,j) - MTX2(i,j);
					}
					else
					{
						retVal = ERR_PARAM_SIZE;
					}
					break;
				case MTX_MULT:
					if( mtx1_->NumCols == mtx2_->NumRows ) //dimensions must agree
					{
						MTXRes(i,j) = 0;  //to avoid overwriting
						for(k = 0u; k < mtx1_->NumCols; k++)
						{
							MTXRes(i,j) += MTX1(i,k) * MTX2(k,j);
						}
					}
					else
					{
						retVal = ERR_PARAM_SIZE;
					}
					break;
				case MTX_SCALE_UP:
						MTXRes(i,j) = MTXRes(i,j) << nScale_;
					break;
				case MTX_SCALE_DOWN:
						MTXRes(i,j) = MTXRes(i,j) >> nScale_;
					break;
				case MTX_TRNS:
						MTXRes(j,i) = MTX1(i,j);
					break;
				case MTX_FILL:
						MTXRes(i,j) = (int32_t)nScale_;
					break;
				case MTX_FILL_DIAG:
					if( i == j )
					{
						MTXRes(i,j) = (int32_t)nScale_;
					}
					else
					{
						MTXRes(i,j) = 0;
					}
					break;
				case MTX_COPY:
					if( (mtx1_->NumCols == mtxRes_->NumCols) && (mtx1_->NumRows == mtxRes_->NumRows) )
					{
						MTXRes(i,j) = MTX1(i,j);
					}
					else
					{
						retVal = ERR_PARAM_SIZE;
					}
					break;
				default:
					retVal |= ERR_PARAM_DATA;
					break;
				}
			}
		}
	}
	return retVal;
}

static inline void MtxOverFlowMult(const int32_t fac1_, const int32_t fac2_, int32_t *prod_, uint8_t *nRightShift_)
{
	/* Value information */
	int8_t  sig_a = 0u, sig_b = 0u;
	int32_t mag_a = 0,  mag_b = 0;

	/* Shifts */
	uint8_t tz_maga = 0u, tz_magb=0u;
	uint8_t lz_maga = 0u, lz_magb=0u;
	uint8_t extraShift = 0u;

	sig_a = (int8_t) SIGN( fac1_ );
	sig_b = (int8_t) SIGN( fac2_ );

	mag_a = (int32_t) sig_a * fac1_;
	mag_b = (int32_t) sig_b * fac2_;

	tz_maga = ctz(mag_a);
	tz_magb = ctz(mag_b);

	mag_a = mag_a >> tz_maga;
	mag_b = mag_b >> tz_magb;

	lz_maga = clz(mag_a);
	lz_magb = clz(mag_b);

	for(extraShift = 0u; 32 >= (lz_maga + lz_magb); extraShift++)
	{
		if(mag_a > mag_b)
		{
			mag_a = mag_a >> 1;
			lz_maga++;
		}
		else
		{
			mag_b = mag_b >> 1;
			lz_magb++;
		}
	}

	if( 0 > (sig_a * sig_b) )
	{
		*prod_ = -(mag_a * mag_b);
	}
	else
	{
		*prod_ = (mag_a * mag_b);
	}
	*nRightShift_ = (tz_maga + tz_magb + extraShift);
}


/* nScaleU_  determines precision of U... 16 seems reasonable */
static inline StdRtn_t MTXUdDecomposition(const MTX_t *mtx_, MTX_t *mtxu_, MTX_t *mtxd_, const uint8_t nScaleU_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	/* Loop variables   ATTENTION: i,j need to be of signed data type */
	int16_t i = 0, j = 0, k = 0;
	uint8_t dim = mtx_->NumCols;

	uint8_t rightShiftuikdkk = 0u;
	uint8_t rightShiftuikdkkujk = 0u;

	int32_t sigma = 0;
	int32_t uikdkk = 0;
	int32_t uikdkkujk = 0;


	if( (NULL != mtxu_) && (NULL != mtxd_) )
	{
		retVal = ERR_OK;
		for(j = (dim-1); j >= 0; j--)
		{
			for(i = j; i >= 0; i--)
			{
				sigma = MTX_ij(mtx_, i, j);
				for(k = (j+1); k < dim; k++)
				{
					MtxOverFlowMult( MTX_ij(mtxu_, i, k), MTX_ij(mtxd_, k, k), &uikdkk,    &rightShiftuikdkk );
					MtxOverFlowMult( uikdkk,              MTX_ij(mtxu_, j, k), &uikdkkujk, &rightShiftuikdkkujk);

					uikdkkujk = uikdkkujk >> ( (2 * nScaleU_) - (rightShiftuikdkk + rightShiftuikdkkujk) );
					sigma -= uikdkkujk;
				}
				if( i == j )
				{
					MTX_ij(mtxd_, j, j) = sigma;
					MTX_ij(mtxu_, j, j) = 1 << nScaleU_;
				}
				else
				{
					MTX_ij(mtxu_, i, j) = (sigma << nScaleU_) / MTX_ij(mtxd_, j, j);
					MTX_ij(mtxu_, j, i) = 0;
				}
			}
		}
	}
	return retVal;
}

static inline StdRtn_t MtxCountLeadingZeros(const MTX_t *mtx_, uint8_t *nLdngZrs_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u, j = 0u;
	uint8_t tmpCnt = 0xFFu;
	uint8_t nZeros = 0u;

	if( NULL != nLdngZrs_ )
	{
		retVal = ERR_OK;
		for(i = 0u; i < mtx_->NumRows; i++)
		{
			for(j = 0u; j < mtx_->NumCols; j++)
			{
				nZeros = clz( MTX_ij(mtx_, i, j) );
				if( nZeros < tmpCnt )
				{
					tmpCnt = nZeros;
				}
			}
		}
		*nLdngZrs_ = tmpCnt;
	}
	return retVal;
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t MTX_Add(const MTX_t *smd1_, const MTX_t *smd2_, MTX_t *sum_)
{
	return MtxCalc(smd1_, smd2_, MTX_ADD, sum_, 0);
}

StdRtn_t MTX_Sub(const MTX_t *min_, const MTX_t *sub_, MTX_t *diff_)
{
	return MtxCalc(min_, sub_, MTX_SUB, diff_, 0);
}

StdRtn_t MTX_Mult(const MTX_t *fac1_, const MTX_t *fac2_, MTX_t *prod_)
{
	return MtxCalc(fac1_, fac2_, MTX_MULT, prod_, 0);
}

StdRtn_t MTX_ScaleUp(MTX_t *mtx_, const uint8_t nScale_)
{
	return MtxCalc(mtx_, mtx_, MTX_SCALE_UP, mtx_, nScale_);
}

StdRtn_t MTX_ScaleDown(MTX_t *mtx_, const uint8_t nScale_)
{
	return MtxCalc(mtx_, mtx_, MTX_SCALE_DOWN, mtx_, nScale_);
}

StdRtn_t MTX_Transpose(const MTX_t *mtx_, MTX_t *mtxTrnspsd_)
{
	return MtxCalc(mtx_, mtx_, MTX_TRNS, mtxTrnspsd_, 0);
}

StdRtn_t MTX_Fill(MTX_t *mtx_, const uint8_t val_)
{
	return MtxCalc(mtx_, mtx_, MTX_FILL, mtx_, val_);
}

StdRtn_t MTX_FillDiagonal(MTX_t *mtx_, const uint8_t val_)
{
	return MtxCalc(mtx_, mtx_, MTX_FILL_DIAG, mtx_, val_);
}

StdRtn_t MTX_Copy(const MTX_t *mtx1_, MTX_t *mtx2_)
{
	return MtxCalc(mtx1_, mtx2_, MTX_COPY, mtx2_, 0);
}

StdRtn_t MTX_UdDecomposition(const MTX_t * mtx_, MTX_t *mtxu_, MTX_t *mtxd_, const uint8_t nScale_)
{
	return MTXUdDecomposition(mtx_, mtxu_, mtxd_, nScale_);
}

StdRtn_t MTX_CountLeadingZeros(const MTX_t *mtx_, uint8_t *nLeadingZeros_)
{
	return MtxCountLeadingZeros(mtx_, nLeadingZeros_);
}



#ifdef MASTER_mtx_C_
#undef MASTER_mtx_C_
#endif /* !MASTER_mtx_C_ */



