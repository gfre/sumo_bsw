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
	,MTX_CNT_OF_OPS
}MTX_Op_t;


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



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
				default:
					retVal |= ERR_PARAM_DATA;
					break;
				}
			}
		}
	}
	return retVal;


#define DFLT_U_SCALING (16)
static inline StdRtn_t MTXUdDecomp(const MTX_t *mtx_, MTX_t *mtxu_, MTX_t *mtxd_, const uint8_t nScale_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int16_t i = 0, j = 0, k = 0;
	int32_t sigma = 0;
	uint8_t m = mtx_->NumCols;

	int8_t  sig_uik = 0u, sig_dkk = 0u, sig_ujk = 0u;
	int32_t mag_uik = 0,  mag_dkk = 0,  mag_ujk = 0;
	uint8_t tz_maguik = 0u, tz_magdkk=0u, tz_magujk=0u;
	uint8_t lz_maguik = 0u, lz_magdkk=0u, lz_magujk=0u;
	int32_t tempProd = 0;
	uint8_t lz_tempProd = 0;
	uint8_t tz_tempProd = 0;
	int32_t sigma_hat = 0;

	if( (NULL != mtxu_) && (NULL != mtxd_) )
	{
		retVal = ERR_OK;
		for(j = (m-1); j >= 0; j--)
		{
			for(i = j; i >= 0; i--)
			{
				sigma = MTX_ij(mtx_, i, j);
				for(k = (j+1); k < m; k++)
				{
					sig_uik = (int8_t) SIGN( MTX_ij(mtxu_, i, k) );
					sig_dkk = (int8_t) SIGN( MTX_ij(mtxd_, k, k) );
					sig_ujk = (int8_t) SIGN( MTX_ij(mtxu_, j, k) );

					mag_uik = (int32_t) sig_uik * MTX_ij(mtxu_, i, k);
					mag_dkk = (int32_t) sig_dkk * MTX_ij(mtxd_, k, k);
					mag_ujk = (int32_t) sig_ujk * MTX_ij(mtxu_, j, k);

					tz_maguik = ctz(mag_uik);
					tz_magdkk = ctz(mag_dkk);
					tz_magujk = ctz(mag_ujk);

					mag_uik = mag_uik >> tz_maguik;
					mag_dkk = mag_dkk >> tz_magdkk;
					mag_ujk = mag_ujk >> tz_magujk;

					lz_maguik = clz(mag_uik);
					lz_magdkk = clz(mag_dkk);
					lz_magujk = clz(mag_ujk);

					if( 32 < (lz_maguik + lz_magdkk) )
					{
						tempProd = mag_uik *  mag_dkk;
					}
					else
					{
						/* TODO shift further to the right?! */
					}

					tz_tempProd = ctz(tempProd);
					tempProd = tempProd >> tz_tempProd;
					lz_tempProd = clz(tempProd);

					if( 32 < (lz_tempProd + lz_magujk) )
					{
						sigma_hat = tempProd * mag_ujk;
					}
					else
					{
						/* TODO shift further to the right?! */
					}

					sigma_hat = sigma_hat >> ( (2*DFLT_U_SCALING) - (tz_maguik + tz_magdkk + tz_tempProd + tz_magujk));  /* TODO equation might be wrong */

					if( 1 == (sig_uik * sig_dkk * sig_ujk) )
					{
						sigma -= sigma_hat;
					}
					else
					{
						sigma += sigma_hat;
					}
				}
				if( i == j )
				{
					MTX_ij(mtxd_, j, j) = sigma;
					MTX_ij(mtxu_, j, j) = 1 << DFLT_U_SCALING;
				}
				else
				{
					MTX_ij(mtxu_, i, j) = (sigma << DFLT_U_SCALING) / MTX_ij(mtxd_, j, j);
				}
			}
		}
	}
	return retVal;
}

static inline StdRtn_t MtxCntLdngZrs(const MTX_t *mtx_, uint8_t *nLdngZrs_)
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

StdRtn_t MTX_FillDiag(MTX_t *mtx_, const uint8_t val_)
{
	return MtxCalc(mtx_, mtx_, MTX_FILL_DIAG, mtx_, val_);
}

StdRtn_t MTX_UdDecomp(const MTX_t * mtx_, MTX_t *mtxu_, MTX_t *mtxd_, const uint8_t nScale_)
{
	return MTXUdDecomp(mtx_, mtxu_, mtxd_, nScale_);
}

StdRtn_t MTX_CountLeadingZeros(const MTX_t *mtx_, uint8_t *nLeadingZeros_)
{
	return MtxCntLdngZrs(mtx_, nLeadingZeros_);
}



#ifdef MASTER_mtx_C_
#undef MASTER_mtx_C_
#endif /* !MASTER_mtx_C_ */



