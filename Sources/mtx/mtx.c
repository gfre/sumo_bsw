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
#define MTX_SIZE_ROW (0x02u)
#define MTX_SIZE_COL (0x02u)



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum MTX_Op_e
{
	 MTX_ADD = 0
	,MTX_SUB
	,MTX_MULT
	,MTX_DIV
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
static inline StdRtn_t Div(int32_t x1_,  int32_t x2_, int32_t *res_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static MTX_OpFct_t *opFctHdls[MTX_CNT_OF_OPS] = {Add, Sub, Mult, Div};



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

static inline StdRtn_t Div(int32_t x1_,  int32_t x2_, int32_t *res_)
{
	//TODO
	return ERR_OK;
}

static inline StdRtn_t Trns(int32_t x1_, int32_t x2_, int32_t *res_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != res_)
	{
		*res_  = x1_;
		retVal = ERR_OK;
	}
	return retVal;
}

static inline StdRtn_t LSE()
{

}



static inline StdRtn_t MtxCalc(const MTX_t mtx1_, const MTX_t mtx2_, MTX_Op_t op_, MTX_t* mtxRes_)
{
	uint8_t i=0u, j=0u, k=0u;
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if ( ( NULL != opFctHdls[op_] )  && ( NULL != mtxRes_ ) )
	{
		retVal = ERR_OK;
		for(i = 0; (i < mtx1_.NumRows) && (ERR_OK == retVal); i++)
		{
			for(j = 0; (j < mtx2_.NumCols) && (ERR_OK == retVal); j++)
			{
				switch(op_)
				{
				case MTX_ADD:
				case MTX_SUB:
					if( (mtx1_.NumRows == mtx2_.NumRows) && (mtx1_.NumCols == mtx2_.NumCols) )  //dimensions must agree
					{
						retVal |= opFctHdls[op_]((mtx1_.pData + i*mtx1_.NumCols)[j], (mtx2_.pData + i*mtx2_.NumCols)[j], &( (mtxRes_->pData + i*mtxRes_->NumCols)[j] ));
					}
					else
						retVal = ERR_PARAM_SIZE;
				case MTX_MULT:
					if( mtx1_.NumCols == mtx2_.NumRows ) //dimensions must agree
					{
						(mtxRes_->pData + i*mtxRes_->NumCols)[j] = 0;  //to avoid overwriting
						for(k = 0u; k < mtx1_.NumCols; k++)
						{
							retVal |= opFctHdls[op_]( (mtx1_.pData + i*mtx1_.NumCols)[k], (mtx2_.pData + k*mtx2_.NumCols)[j], &( (mtxRes_->pData + i*mtxRes_->NumCols)[j]) );
						}
					}
					else
						retVal = ERR_PARAM_SIZE;
					break;
				case MTX_DIV:
					break;
//				case MTX_TRNS:
//						retVal |= opFctHdls[op_]( (mtx1_->pData + j*mtx1_->NumCols)[i], 0, &( (mtxRes_->pData + i*mtxRes_->NumCols)[j]));
//					break;
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
StdRtn_t MTX_Add(const MTX_t smd1_, const MTX_t smd2_, MTX_t* sum_)
{
	return MtxCalc(smd1_, smd2_, MTX_ADD, sum_);
}

StdRtn_t MTX_Sub(const MTX_t min_, const MTX_t sub_, MTX_t* diff_)
{
	return MtxCalc(min_, sub_, MTX_SUB, diff_);
}

StdRtn_t MTX_Mult(const MTX_t fac1_, const MTX_t fac2_, MTX_t* prod_)
{
	return MtxCalc(fac1_, fac2_, MTX_MULT, prod_);
}

StdRtn_t MTX_Div(const MTX_t divd_, const MTX_t divs_, MTX_t* quot_)
{
	return MtxCalc(divd_, divs_, MTX_DIV, quot_);
}

//StdRtn_t MTX_Trns(uint8_t  sizeRows_, uint8_t sizeCols_, const MTX_t trns_, const MTX_t null_, MTX_t* trnsp_)
//{
//	return MtxCalc(sizeRows_, sizeCols_, trns_, 0, MTX_TRNS, trnsp_);
//}


#ifdef MASTER_mtx_C_
#undef MASTER_mtx_C_
#endif /* !MASTER_mtx_C_ */
