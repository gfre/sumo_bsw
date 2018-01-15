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
	,MTX_SHIFT_LEFT
	,MTX_SHIFT_RIGHT
	,MTX_TRNS
	,MTX_FILL
	,MTX_FILL_DIAG
	,MTX_COPY
	,MTX_CNT_OF_OPS
}MTX_Op_t;


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static inline StdRtn_t MtxCalc(const MTX_t *mtx1_, const MTX_t *mtx2_, MTX_Op_t op_, MTX_t *mtxRes_, uint8_t nScale_);
static inline StdRtn_t MtxOverFlowMult(const int32_t fac1_, const int32_t fac2_, int32_t *prod_, uint8_t *nRightShifts_);
static inline StdRtn_t MtxUdDecomposition(const MTX_t *mtx_, MTX_t *mtxu_, MTX_t *mtxd_, const uint8_t nScaleU_);
static inline StdRtn_t MtxCountLeadingZeros(MTX_t *mtx_, uint8_t *nLdngZrs_);

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
					MTXRes(i,j) = (MTX1(i,j) + MTX2(i,j));
					break;
				case MTX_SUB:
					MTXRes(i,j) =  (MTX1(i,j) - MTX2(i,j));
					break;
				case MTX_MULT:
					MTXRes(i,j) = 0;  //to avoid overwriting
					for(k = 0u; k < mtx1_->NumCols; k++)
					{
						MTXRes(i,j) += (MTX1(i,k) * MTX2(k,j));
					}
					break;
				case MTX_SHIFT_LEFT:
					MTXRes(i,j) = (MTXRes(i,j) << nScale_);
					break;
				case MTX_SHIFT_RIGHT:
					MTXRes(i,j) = (MTXRes(i,j) >> nScale_);
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
					MTXRes(i,j) = MTX1(i,j);
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

static inline StdRtn_t MtxOverFlowMult(const int32_t fac1_, const int32_t fac2_, int32_t *prod_, uint8_t *nRightShifts_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( (NULL!= prod_) && (NULL != nRightShifts_) )
		{
		retVal = ERR_OK;
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
		*nRightShifts_ = (tz_maga + tz_magb + extraShift);
	}
	return retVal;
}

/* nScaleU_  determines precision of U... 16 seems reasonable */
static inline StdRtn_t MtxUdDecomposition(const MTX_t *mtx_, MTX_t *mtxu_, MTX_t *mtxd_, const uint8_t nScaleU_)
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
				for(k = (j+1); (k < dim) && (ERR_OK == retVal); k++)
				{
					retVal |= MtxOverFlowMult( MTX_ij(mtxu_, i, k), MTX_ij(mtxd_, k, k), &uikdkk,    &rightShiftuikdkk );
					retVal |= MtxOverFlowMult( uikdkk,              MTX_ij(mtxu_, j, k), &uikdkkujk, &rightShiftuikdkkujk);

					uikdkkujk = uikdkkujk >> ( (nScaleU_ << 1) - (rightShiftuikdkk + rightShiftuikdkkujk) );
					sigma -= uikdkkujk;
				}
				if( i == j )
				{
					MTX_ij(mtxd_, j, j) = sigma;
					MTX_ij(mtxu_, j, j) = (1 << nScaleU_);
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

static inline StdRtn_t MtxCountLeadingZeros(MTX_t *mtx_, uint8_t *nLdngZrs_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u, j = 0u;
	uint8_t tmpCnt = 0xFFu;
	uint8_t nZeros = 0u;

	int8_t  sig = 0u;
	int32_t mag = 0;

	if( NULL != mtx_ )
	{
		retVal = ERR_OK;
		for(i = 0u; i < mtx_->NumRows; i++)
		{
			for(j = 0u; j < mtx_->NumCols; j++)
			{
				sig = (int8_t) SIGN( MTX_ij(mtx_, i, j) );
				mag = (int32_t) sig * MTX_ij(mtx_, i, j);
				nZeros = clz( mag );
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
StdRtn_t MTX_Add(MTX_t *smd1_, MTX_t *smd2_, MTX_t *sum_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != sum_ )
	{
		retVal = ERR_PARAM_SIZE;
		if(    (   (smd1_->NumRows == smd2_->NumRows)
				&& (smd1_->NumCols == smd2_->NumCols) )
			&& (   (smd1_->NumRows == sum_->NumRows)
				&& (smd1_->NumCols == sum_->NumCols) ) )
		{
			retVal  = ERR_OK;
			retVal |= MtxCalc(smd1_, smd2_, MTX_ADD, sum_, 0);
		}
	}
	return retVal;
}

StdRtn_t MTX_Sub(MTX_t *min_, MTX_t *sub_, MTX_t *diff_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != diff_ )
	{
		retVal = ERR_PARAM_SIZE;
		if(    (   (min_->NumRows == sub_->NumRows)
				&& (min_->NumCols == sub_->NumCols) )
			&& (   (min_->NumRows == diff_->NumRows)
				&& (min_->NumCols == diff_->NumCols) ) )
		{
			retVal  = ERR_OK;
			retVal |= MtxCalc(smd1_, smd2_, MTX_SUB, sum_, 0);
		}
	}
	return retVal;
}

StdRtn_t MTX_Mult(MTX_t *fac1_, MTX_t *fac2_, MTX_t *prod_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != prod_ )
	{
		retVal = ERR_PARAM_SIZE;
		if(    (fac1_->NumCols == fac2_->NumRows)
			&& (prod_->NumRows == fac1_->NumRows)
			&& (prod_->NumCols == fac2_->NumCols)   )
		{
			retVal = ERR_OK;
			retVal |= MtxCalc(fac1_, fac2_, MTX_MULT, prod_, 0);
		}
	}
	return retVal;
}

StdRtn_t MTX_ShiftLeft(MTX_t *mtx_, const uint8_t nScale_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != mtx_ )
	{
		retVal  = ERR_OK;
		retVal |= MtxCalc(mtx_, mtx_, MTX_SHIFT_LEFT, mtx_, nScale_);
	}
	return retVal;
}

StdRtn_t MTX_ShiftRight(MTX_t *mtx_, const uint8_t nScale_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != mtx_ )
	{
		retVal  = ERR_OK;
		retVal |= MtxCalc(mtx_, mtx_, MTX_SHIFT_RIGHT, mtx_, nScale_);
	}
	return retVal;
}

StdRtn_t MTX_Transpose(const MTX_t *mtx_, MTX_t *mtxTrnspsd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != mtxTrnspsd_ )
	{
		retVal = ERR_PARAM_SIZE;
		if( (mtx_->NumRows == mtxTrnspsd_->NumRows)
			&& (mtx_->NumCols == mtxTrnspsd_->NumCols) )
		{
			retVal  = ERR_OK;
			retVal |= MtxCalc(mtx_, mtx_, MTX_TRNS, mtxTrnspsd_, 0);
		}
	}
	return retVal;
}

StdRtn_t MTX_Fill(MTX_t *mtx_, const uint8_t val_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != mtx_ )
	{
		retVal = ERR_OK;
		retVal |= MtxCalc(mtx_, mtx_, MTX_FILL, mtx_, val_);
	}
	return retVal;
}

StdRtn_t MTX_FillDiagonal(MTX_t *mtx_, const uint8_t val_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != mtx_ )
	{
		retVal = ERR_OK;
		retVal |= MtxCalc(mtx_, mtx_, MTX_FILL_DIAG, mtx_, val_);
	}
	return retVal;
}

StdRtn_t MTX_Copy(MTX_t *mtx1_, MTX_t *mtx2_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != mtx2_ )
	{
		retVal = ERR_PARAM_SIZE;
		if( (mtx1_->NumCols == mtx2_->NumCols) && (mtx1_->NumRows == mtx2_->NumRows) )
		{
			retVal  = ERR_OK;
			retVal |= MtxCalc(mtx1_, mtx2_, MTX_COPY, mtx2_, 0);
		}
	}
	return retVal;
}

StdRtn_t MTX_UdDecomposition(const MTX_t *mtx_, MTX_t *mtxu_, MTX_t *mtxd_, const uint8_t nScaleU_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( (NULL != mtxu_) && (NULL != mtxd_) )
	{
		retVal  = ERR_OK;
		retVal |= MTXUdDecomposition(mtx_, mtxu_, mtxd_, nScaleU_);
	}
	return retVal;
}

StdRtn_t MTX_OverFlowMult(const int32_t fac1_, const int32_t fac2_, int32_t *prod_, uint8_t *nRightShift)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( (NULL!= prod_) && (NULL != nRightShifts_) )
	{
		retVal  = ERR_OK;
		retVal |= MtxOverFlowMult(fac1_, fac2_, prod_, nRightShifts_);
	}
	return retVal;
}

StdRtn_t MTX_CountLeadingZeros(MTX_t *mtx_, uint8_t *nLdngZrs_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( (NULL != mtx_) && (NULL != nLdngZrs_) )
	{
		retVal  = ERR_OK;
		retVal |= MtxCountLeadingZeros(mtx_, nLdngZrs_);
	}
	return retVal;
}


#ifdef MASTER_mtx_C_
#undef MASTER_mtx_C_
#endif /* !MASTER_mtx_C_ */
