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
#define MTX_ij(mtx_, i_, j_) ( mtx_->pData[i_*mtx_->NumCols + j_] )
#define MTXLOC_ij(mtx_, i_, j_) ( mtx_.pData[i_*mtx_.NumCols + j_] )

#define MTXROW_i(mtx_, i_) ( mtx_->pData[i_*mtx_->NumCols] )
#define MTXLOCROW_i(mtx_, i_) ( mtx_.pData[i_*mtx_.NumCols] )

#define VEC_ij(vec_, i_, j_) ( vec_->pData[i_*vec_->NumCols + j_] )

#define MTX1(i_, j_) ( MTX_ij(mtx1_, i_, j_) )
#define MTX2(i_, j_) ( MTX_ij(mtx2_, i_, j_) )
#define MTXRes(i_, j_) ( MTX_ij(mtxRes_, i_, j_) )

#define MTXAug(i_, j_) ( MTXLOC_ij(AugmentedMat, i_, j_) )
#define MTX1Row(i_) ( MTXROW_i(mtx1_, (i_)) )
#define MTXAugRow(i_) ( MTXLOCROW_i(AugmentedMat, (i_)) )

#define VEC1(i_, j_) ( VEC_ij(vec1_, i_, j_) )
#define VECRes(i_,j_) ( VEC_ij(vecRes_, i_, j_) )



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum MTX_Op_e
{
	 MTX_ADD = 0
	,MTX_SUB
	,MTX_MULT
	,MTX_SCALE_UP
	,MTX_SCALE_DOWN
//	,MTX_TRNS
//	,MTX_INV
//	,MTX_ADJ
	,MTX_CNT_OF_OPS
}MTX_Op_t;


typedef StdRtn_t MTX_OpFct_t(int32_t x1_,  int32_t x2_, int32_t *res_);

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static inline StdRtn_t Add(int32_t x1_,  int32_t x2_, int32_t *res_);
static inline StdRtn_t Sub(int32_t x1_,  int32_t x2_, int32_t *res_);
static inline StdRtn_t Mult(int32_t x1_, int32_t x2_, int32_t *res_);
static inline StdRtn_t ScaleUp(int32_t x1_, int32_t nScale_, int32_t *res_);
static inline StdRtn_t ScaleDown(int32_t x1_, int32_t nScale_, int32_t *res_);




/*=================================== >> GLOBAL VARIABLES << =====================================*/
static MTX_OpFct_t *opFctHdls[MTX_CNT_OF_OPS] = {Add, Sub, Mult, ScaleUp, ScaleDown};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static inline StdRtn_t Add(int32_t x1_,  int32_t x2_, int32_t *res_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != res_)
	{
		*res_  = x1_ + x2_;
		retVal = ERR_OK;
	}
	return retVal;
}

static inline StdRtn_t Sub(int32_t x1_,  int32_t x2_, int32_t *res_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != res_)
	{
		*res_  = x1_ - x2_;
		retVal = ERR_OK;
	}
	return retVal;
}

static inline StdRtn_t Mult(int32_t x1_,  int32_t x2_, int32_t *res_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != res_)
	{
		*res_ += x1_ * x2_;
		retVal = ERR_OK;
	}
	return retVal;
}

static inline StdRtn_t ScaleUp(int32_t x1_, int32_t nScale_, int32_t *res_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	nScale_ = (uint8_t)nScale_;
	if (NULL != res_)
	{
		*res_ = *res_ << nScale_;
		retVal = ERR_OK;
	}
	return retVal;
}

static inline StdRtn_t ScaleDown(int32_t x1_, int32_t nScale_, int32_t *res_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	nScale_ = (uint8_t)nScale_;
	if (NULL != res_)
	{
		*res_ = *res_ >> nScale_;
		retVal = ERR_OK;
	}
	return retVal;
}

static inline StdRtn_t AppndColVecToMat(const MTX_t *mtx1_, const MTX_t *vec1_, MTX_t *mtxRes_)
{
	uint8_t i = 0u, j = 0u;
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if ( NULL != mtxRes_)
	{
		retVal = ERR_OK;
		for(i = 0u; i < mtxRes_->NumRows; i++)
		{
			for(j = 0u; j < mtxRes_->NumCols; j++)
			{
				if( j == mtx1_->NumCols )
					MTXRes(i,j) = VEC1(i,0);
				else
					MTXRes(i,j) = MTX1(i,j);
			}
		}
	}
	return retVal;
}

static inline StdRtn_t DvdBySmllstValInRow(int32_t *row1_, uint8_t rowLen_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t temp = 0;
	uint8_t j = 0u;
	if(NULL != row1_)
	{
		retVal = ERR_OK;
		int32_t minVal = 0x7FFFFFFF;
		for (j = 0u; j < rowLen_; j++)
		{
			if(row1_[j] < 0)  /* change sign if value is negative */
			{
				temp = -row1_[j];
			}
			else
			{
				temp = row1_[j];
			}
			if( (temp < minVal) && (temp != 0) ) 	/* find smallest value in row that is not zero */
			{

				minVal = temp;
			}
		}
		for (j = 0u; j < rowLen_; j++)
		{
			if(minVal != 1)		/* leave row as is if smallest value is 1 */
			{
				row1_[j] /=  minVal;    /* divide entire row by smallest value */
			}
		}
	}
	return retVal;
}

static inline StdRtn_t FindLeadingCoeffIdxInRow(int32_t *row_, uint8_t rowLen_, uint8_t *res_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t j = 0u;
	if(NULL != res_)
	{
		retVal = ERR_OK;
		*res_ = 0u;
		bool hasPivot = FALSE;
		for(j = 0u; (j < rowLen_) && (FALSE == hasPivot); j++)
		{
			if( 0 != row_[j] )
			{
				*res_ = j;
				hasPivot = TRUE;
			}
		}
	}
	return retVal;
}

static inline StdRtn_t FindLargestValInRow(int32_t *row_, uint8_t rowLen_, int32_t* largestValInRow_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t j = 0u;
	int32_t largestNeg = 0, largestPos = 0;
	if(NULL != largestValInRow_)
	{
		retVal = ERR_OK;
		for(j = 0u; j < rowLen_; j++)
		{
			if(row_[j] > largestPos)
				largestPos = row_[j];
			if(row_[j] < largestNeg)
				largestNeg = row_[j];
		}
	if( (-largestNeg) > largestPos )
		*largestValInRow_ = -largestNeg;
	else
		*largestValInRow_ = largestPos;
	}
	return retVal;
}

static inline StdRtn_t MultRowWFctr(int32_t *row_, uint8_t rowLen_, int32_t fac_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t j = 0u;
	if(NULL != row_)
	{
		retVal = ERR_OK;
		for(j = 0u; j < rowLen_; j++)
		{
			row_[j] *= fac_;
		}
	}
	return retVal;
}

static inline StdRtn_t SbtrctRow1frmRow2(int32_t *row1_, int32_t *row2_, uint8_t rowLen_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t j = 0u;
	if( (NULL != row1_) && (NULL != row2_))
	{
		retVal = ERR_OK;
		for(j = 0u; j < rowLen_; j++)
		{
			row2_[j] -= row1_[j];
		}
	}
	return retVal;
}

static inline StdRtn_t bSortRow(uint8_t *rowToSort_, uint8_t *aIdx_, uint8_t rowLen_)
{
	uint8_t i = 0u, j = 0u;
	int32_t temp = 0, tmpIdx = 0;
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != rowToSort_)
	{
		retVal = ERR_OK;
		for (i = 0u ;  i < (rowLen_); i++)
		{
			for (j = 0u ; j < (rowLen_-i); j++)
			{
				if (rowToSort_[j] > rowToSort_[j+1])
				{
					temp      		= rowToSort_[j]; //swap values
					rowToSort_[j]   = rowToSort_[j+1];
					rowToSort_[j+1] = temp;
					tmpIdx	   = aIdx_[j]; 	//swap indices
					aIdx_[j]   = aIdx_[j+1];
					aIdx_[j+1] = tmpIdx;
				}
			}
		}
	}
	return retVal;
}

static inline StdRtn_t SlvSystmOfLnrEqtns(const MTX_t *mtx1_ , const MTX_t *vec1_, MTX_t *vecRes_, uint8_t nScale_)
{
	uint8_t i = 0u, j = 0u;
	uint8_t aLeadCoeff[mtx1_->NumRows];
	uint8_t aIdx[mtx1_->NumRows];
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t tempMat[mtx1_->NumRows][mtx1_->NumCols + 1];
	int32_t	largestValInRow = 0;
	int32_t a = 0, b = 0;
	MTX_t AugmentedMat = {tempMat[0], mtx1_->NumRows, (mtx1_->NumCols + 1)};

	if(NULL != vecRes_)
	{
		retVal = ERR_OK;
		if( (mtx1_->NumCols == vec1_->NumRows) && (vec1_->NumRows == vecRes_->NumRows) && (vec1_->NumCols == vecRes_->NumCols) ) //dimensions of inputs must fit
		{
			/* Initialisation */
			retVal |= AppndColVecToMat(mtx1_, vec1_, &AugmentedMat);
			for(i = 0u; i < (mtx1_->NumRows) && (ERR_OK == retVal); i++)
			{
				aIdx[i] = i;
				retVal |= FindLeadingCoeffIdxInRow(&MTXAugRow(i), AugmentedMat.NumCols, &aLeadCoeff[i]);
				VECRes(i,0) = 0;
			}
			retVal |= bSortRow(aLeadCoeff, aIdx, AugmentedMat.NumRows);

			/* Gauss algorithm */
			while( (aLeadCoeff[AugmentedMat.NumRows-1] != (AugmentedMat.NumRows-1)) && (ERR_OK == retVal) )
			{
				for(i = 0u; i < (AugmentedMat.NumRows-1); i++)
				{
					if(aLeadCoeff[i] == aLeadCoeff[i+1])
					{
						a = MTXAug(aIdx[i], aLeadCoeff[i]);
						b = MTXAug(aIdx[i+1], aLeadCoeff[i+1]);
						retVal |= MultRowWFctr(&MTXAugRow(aIdx[i]), AugmentedMat.NumCols, b);
						retVal |= MultRowWFctr(&MTXAugRow(aIdx[i+1]), AugmentedMat.NumCols, a);
						retVal |= SbtrctRow1frmRow2(&MTXAugRow(aIdx[i]), &MTXAugRow(aIdx[i+1]), AugmentedMat.NumCols);
						for(j = 0u; (j < AugmentedMat.NumRows) && (ERR_OK == retVal); j++)
						{

							retVal |= FindLeadingCoeffIdxInRow(&MTXAugRow(aIdx[j]), AugmentedMat.NumCols, &aLeadCoeff[j]);
							retVal |= FindLargestValInRow(&MTXAugRow(j), AugmentedMat.NumCols, &largestValInRow);
							if(largestValInRow > 500000)
							{
								retVal |= DvdBySmllstValInRow(&MTXAugRow(j), AugmentedMat.NumCols);
							}
						}
						retVal |= bSortRow(aLeadCoeff, aIdx, AugmentedMat.NumCols);
					}
				}
			}
		/* Backstepping */
			for(i = AugmentedMat.NumRows; i >= 1; i--)
			{
				int32_t sum = 0;
				for(j = (i - 1); j<AugmentedMat.NumRows; j++)
				{
					sum += MTXAug(aIdx[i-1],j) * VECRes(j,0);
				}
				VECRes(i-1,0) = ( (MTXAug(aIdx[i-1], AugmentedMat.NumRows) << nScale_) - sum) / MTXAug(aIdx[i-1], i-1);
			}
		}else
		{
			retVal = ERR_PARAM_SIZE;
		}
	}
	return retVal;
}

static inline StdRtn_t MtxCalc(const MTX_t *mtx1_, const MTX_t *mtx2_, MTX_Op_t op_, MTX_t *mtxRes_, uint8_t nScale_)
{
	uint8_t i=0u, j=0u, k=0u;
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if ( ( NULL != opFctHdls[op_] )  && ( NULL != mtxRes_ ) )
	{
		retVal = ERR_OK;
		for(i = 0; (i < mtx1_->NumRows) && (ERR_OK == retVal); i++)
		{
			for(j = 0; (j < mtx2_->NumCols) && (ERR_OK == retVal); j++)
			{
				switch(op_)
				{
				case MTX_ADD:
				case MTX_SUB:
					if( ((mtx1_->NumRows == mtx2_->NumRows) && (mtx1_->NumCols == mtx2_->NumCols)) && ((mtxRes_->NumRows == mtx1_->NumRows) && (mtxRes_->NumCols == mtx1_->NumCols))  )  //dimensions must agree
					{
						retVal |= opFctHdls[op_](MTX1(i,j), MTX2(i,j), &(MTXRes(i,j)) );
					}
					else
						retVal = ERR_PARAM_SIZE;
					break;
				case MTX_MULT:
					if( mtx1_->NumCols == mtx2_->NumRows ) //dimensions must agree
					{
						MTXRes(i,j) = 0;  //to avoid overwriting
						for(k = 0u; k < mtx1_->NumCols; k++)
						{
							retVal |= opFctHdls[op_](MTX1(i,k), MTX2(k,j), &(MTXRes(i,j)) );
						}
					}
					else
						retVal = ERR_PARAM_SIZE;
					break;
				case MTX_SCALE_UP:
				case MTX_SCALE_DOWN:
					retVal |= opFctHdls[op_](0, (int32_t)nScale_, &(MTXRes(i,j)) );
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

StdRtn_t MTX_Div(const MTX_t *mat_,const MTX_t *vec_, MTX_t *vecRes_, uint8_t nScale_)
{
	return SlvSystmOfLnrEqtns(mat_, vec_, vecRes_, nScale_);
}


#ifdef MASTER_mtx_C_
#undef MASTER_mtx_C_
#endif /* !MASTER_mtx_C_ */



