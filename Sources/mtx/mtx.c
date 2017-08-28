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

//static inline StdRtn_t FindPivotCoeffInRow(MTX_t *mtx1_, uint8_t row_, uint8_t *pivot_)
//{
//	StdRtn_t retVal = ERR_PARAM_ADDRESS;
//	uint8_t j = 0u;
//
//	if(NULL != pivot_)
//	{
//		retVal = ERR_OK;
//		*pivot_ = 0u;
//		bool hasPivot = FALSE;
//		for(j = 0u; (j < mtx1_->NumCols) && (FALSE == hasPivot); j++)
//		{
//			if( 0 != MTX1(row_, j) )
//			{
//				*pivot_ = j;
//				hasPivot = TRUE;
//			}
//		}
//	}
//	return retVal;
//}

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

static inline StdRtn_t SlvSystmOfLnrEqtns(const MTX_t *mtx1_ , const MTX_t *vec1_, MTX_t *vecRes_, uint8_t nScale_)
{
	uint8_t i = 0u;
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t tempMat[mtx1_->NumRows][mtx1_->NumCols + 1];
	MTX_t AugmentedMat = {tempMat[0], mtx1_->NumRows, (mtx1_->NumCols + 1)};

	if(NULL != vecRes_)
	{
		if( (mtx1_->NumCols == vec1_->NumRows) && (vec1_->NumRows == vecRes_->NumRows) && (vec1_->NumCols == vecRes_->NumCols) )
		{
			retVal = ERR_OK;
			retVal |= AppndColVecToMat(mtx1_, vec1_, &AugmentedMat);
			retVal |= MultRowWFctr(&MTXAugRow(0), AugmentedMat.NumCols, MTX1(1,0));
			retVal |= MultRowWFctr(&MTXAugRow(1), AugmentedMat.NumCols, MTX1(0,0));
			retVal |= SbtrctRow1frmRow2(&MTXAugRow(0), &MTXAugRow(1), AugmentedMat.NumCols);

			retVal |= DvdBySmllstValInRow(&MTXAugRow(0), AugmentedMat.NumCols);
			retVal |= DvdBySmllstValInRow(&MTXAugRow(1), AugmentedMat.NumCols);

			VECRes(1,0) = ( MTXAug(1,2) << nScale_ ) / MTXAug(1,1);
			VECRes(0,0) = ( ( MTXAug(0,2) << nScale_ ) - MTXAug(0,1) * VECRes(1,0) ) / MTXAug(0,0);
		}
		else
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
					retVal |= opFctHdls[op_](0, (int32_t)nScale_, &MTXRes(i,j) );
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



