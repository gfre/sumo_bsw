/*************************************************************************************************
 * @file		kf.c
 * @ingroup		kf Kalman FIlter
 * @brief 		This Module implements a Kalman Filter for the velocity given the current position.
 *
 *		This file contains all the logic for the kalman filtering. It contains functions to deal with matrix operations,
 *		matrix vector operations, and vector operations such as multiplying, transposing, etc. The init function
 *		transposes the A (system) matrix and c (measurement) vector and stores it in a global variable. It also
 *		receives the configuration from kf_cfg.c.
 *		The main function implements a recursive kalman filter algorithm and is currently called as part of the "drive" module.
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	23.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_KF_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "kf.h"
#include "kf_cfg.h"
#include "kf_api.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "tacho_api.h"



/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void KF_UpdateMeasurements(void);
static void KF_UpdateUnscaledVals(void);
static void KF_UpdateModuloCounter(void);
static void KF_SetInitialValues(void);

//Matrix operations
static StdRtn_t KF_AddMatrices(const KF_I32Mat_t* m_, const KF_I32Mat_t* n_, KF_I32Mat_t* result_, bool subtract_);
static StdRtn_t KF_MultMatrices(const KF_I32Mat_t* m_, const KF_I32Mat_t* v_, KF_I32Mat_t* result_, int32_t divider_);
static StdRtn_t KF_TransposeMat(const KF_I32Mat_t* m_, KF_I32Mat_t* result_);
static StdRtn_t KF_InvertMatrix(const KF_I32Mat_t* m_, const int32_t det_m_, KF_I32Mat_t* result_, int32_t scaleDown_);
static StdRtn_t KF_CalcMinor(const KF_I32Mat_t* m_, const uint8_t row_, const uint8_t col_, KF_I32MatLowDim_t* result_);
static StdRtn_t KF_CalcDeterminant(const KF_I32Mat_t* m_, const KF_I32MatLowDim_t* mlow_, int32_t* result_, int32_t scaleDown_);

//Matrix-vector operations
static StdRtn_t KF_MultRowVecMat(const KF_I32RowVec_t* v_, const KF_I32Mat_t* M_, KF_I32RowVec_t* result_);
static StdRtn_t KF_MultMatColVec(const KF_I32Mat_t* m_, const KF_I32ColVec_t* n_, KF_I32ColVec_t* result_, int32_t divider_);

//Vector operations
static StdRtn_t KF_MultRowVecColVec(const KF_I32RowVec_t* v_, const KF_I32ColVec_t* w_, int32_t* result_);
static StdRtn_t KF_TransposeRowVec(const KF_I32RowVec_t* v_, KF_I32ColVec_t* result_);
static StdRtn_t KF_MultColVecFactor(const KF_I32ColVec_t* v_, const int32_t factor_, KF_I32ColVec_t* result_, bool divide_);
static StdRtn_t KF_AddColVecs(const KF_I32ColVec_t* v_,const KF_I32ColVec_t* w_, KF_I32ColVec_t* result_, bool subtract_);
static StdRtn_t KF_MultColVecRowVec(const KF_I32ColVec_t* v_, const KF_I32RowVec_t* w_, KF_I32Mat_t* result_);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static const KF_Cfg_t* kfCfg = NULL;

static KF_I32Mat_t 	  KF_AT = {0};
static KF_I32Mat_t 	  KF_Ident =
{
		{
				{1, 0},
				{0, 1},
		},
};

static KF_I32Mat_t 	  KF_LeftCorrErrInEst  = {0};
static KF_I32Mat_t 	  KF_RightCorrErrInEst = {0};
static KF_I32Mat_t	  KF_LeftPredErrInEst  = {0};
static KF_I32Mat_t	  KF_RightPredErrInEst = {0};

static KF_I32ColVec_t KF_LeftCorrStateEst 	 = {0};
static KF_I32ColVec_t KF_RightCorrStateEst 	 = {0};
static KF_I32ColVec_t KF_LeftPredStateEst 	 = {0};
static KF_I32ColVec_t KF_RightPredStateEst 	 = {0};

static KF_I32ColVec_t KF_LeftU 			= {0};
static KF_I32ColVec_t KF_RightU 		= {0};

static int32_t 		  KF_LeftSpeed  = 0;
static int32_t 		  KF_RightSpeed = 0;
static int32_t 		  KF_LeftPos  = 0;
static int32_t 		  KF_RightPos = 0;

static int16_t KF_LeftModCntr 	  = 0;
static int16_t KF_RightModCntr 	  = 0;

#if KF_USE_MEASUREMENT_MATRIX
	static KF_I32Mat_t KF_CT = {0};

	static KF_I32Mat_t KF_LeftKalmanGain 		= {0};
	static KF_I32Mat_t KF_RightKalmanGain 		= {0};
	static KF_I32ColVec_t KF_LeftResiduum 		= {0};
	static KF_I32ColVec_t KF_RightResiduum 		= {0};
	static KF_I32ColVec_t KF_LeftY 	= {0};
	static KF_I32ColVec_t KF_RightY	= {0};
	static KF_I32Mat_t KF_LeftDenominator  = {0};
	static KF_I32Mat_t KF_RightDenominator = {0};

#else
	static KF_I32ColVec_t KF_c 	= {0};

	static KF_I32ColVec_t KF_LeftKalmanGain  = {0};
	static KF_I32ColVec_t KF_RightKalmanGain = {0};
	static int32_t KF_LeftResiduum 		= 0;
	static int32_t KF_RightResiduum 	= 0;
	static int32_t KF_RightY	= 0;
	static int32_t KF_LeftY		= 0;
	static int32_t KF_RightDenominator 	= 0;
	static int32_t KF_LeftDenominator 	= 0;

#endif



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void KF_UpdateMeasurements()
{
#if KF_USE_MEASUREMENT_MATRIX
		KF_LeftY.aRow[0] +=  ((Q4CLeft_GetPos()*KF_SCALE_X)-(KF_LeftY.aRow[0] + (KF_LeftModCntr*(KF_MAX_POS_VAL/KF_SCALE_A))));
		KF_LeftY.aRow[1] = KF_SCALE_X*TACHO_GetUnfilteredSpeed(TRUE);

		KF_RightY.aRow[0] +=  ((Q4CRight_GetPos()*KF_SCALE_X)-(KF_RightY.aRow[0] + (KF_RightModCntr*(KF_MAX_POS_VAL/KF_SCALE_A))));
		KF_RightY.aRow[1] = 0;
#else
		KF_LeftY  += ((Q4CLeft_GetPos()*KF_SCALE_X)-(KF_LeftY + (KF_LeftModCntr*(KF_MAX_POS_VAL/KF_SCALE_A))));

		KF_RightY += ((Q4CRight_GetPos()*KF_SCALE_X)-(KF_RightY + (KF_RightModCntr*(KF_MAX_POS_VAL/KF_SCALE_A))));
#endif
}

static void KF_UpdateInput()
{
	/*
	 * update your input vector here, e.g:
	 KF_LeftU.aRow[0] = ...;
	 KF_LeftU.aRow[1] = ...;

	 KF_RightU.aRow[0] = ...;
	 KF_RightU.aRow[1] = ...;
	 *
	 */
}

static void KF_UpdateUnscaledVals()
{
	KF_LeftPos = (KF_LeftModCntr*((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) + KF_LeftCorrStateEst.aRow[0])/((int32_t)KF_SCALE_X);
	KF_LeftSpeed = KF_LeftCorrStateEst.aRow[1]/(int32_t)KF_SCALE_X;

	KF_RightPos = (KF_RightModCntr*((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) + KF_RightCorrStateEst.aRow[0])/((int32_t)KF_SCALE_X);
	KF_RightSpeed = KF_RightCorrStateEst.aRow[1]/(int32_t)KF_SCALE_X;
}

static void KF_UpdateModuloCounter()
{
#if KF_USE_MEASUREMENT_MATRIX
	if((KF_LeftCorrStateEst.aRow[0] >= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_LeftY.aRow[0]) >= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
		{
			KF_LeftCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_LeftY.aRow[0]  		%= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_LeftModCntr++;
		}else if((KF_LeftCorrStateEst.aRow[0] <= -(int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_LeftY.aRow[0]) <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
		{
			KF_LeftCorrStateEst.aRow[0] = -KF_LeftCorrStateEst.aRow[0];
			KF_LeftY.aRow[0] 		= -KF_LeftY.aRow[0] ;
			KF_LeftCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_LeftY.aRow[0]  		%= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_LeftCorrStateEst.aRow[0] = -KF_LeftCorrStateEst.aRow[0];
			KF_LeftY.aRow[0]  		= -KF_LeftY.aRow[0] ;
			KF_LeftModCntr--;
		}

		if((KF_RightCorrStateEst.aRow[0] >= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_RightY.aRow[0] ) >= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
		{
			KF_RightCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_RightY.aRow[0]  		 %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_RightModCntr++;
		}else if((KF_RightCorrStateEst.aRow[0] <= -(int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_RightY.aRow[0] ) <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
		{
			KF_RightCorrStateEst.aRow[0] = -KF_RightCorrStateEst.aRow[0];
			KF_RightY.aRow[0]  		 = -KF_RightY.aRow[0] ;
			KF_RightCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_RightY.aRow[0]  		 %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_RightCorrStateEst.aRow[0] = -KF_RightCorrStateEst.aRow[0];
			KF_RightY.aRow[0]  		 = -KF_RightY.aRow[0] ;
			KF_RightModCntr--;
		}
#else
	if((KF_LeftCorrStateEst.aRow[0] >= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_LeftY) >= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
	{
		KF_LeftCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_LeftY 		%= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_LeftModCntr++;
	}else if((KF_LeftCorrStateEst.aRow[0] <= -(int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_LeftY) <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
	{
		KF_LeftCorrStateEst.aRow[0] = -KF_LeftCorrStateEst.aRow[0];
		KF_LeftY 		= -KF_LeftY;
		KF_LeftCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_LeftY 		%= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_LeftCorrStateEst.aRow[0] = -KF_LeftCorrStateEst.aRow[0];
		KF_LeftY 		= -KF_LeftY;
		KF_LeftModCntr--;
	}

	if((KF_RightCorrStateEst.aRow[0] >= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_RightY) >= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
	{
		KF_RightCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_RightY 		 %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_RightModCntr++;
	}else if((KF_RightCorrStateEst.aRow[0] <= -(int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_RightY) <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
	{
		KF_RightCorrStateEst.aRow[0] = -KF_RightCorrStateEst.aRow[0];
		KF_RightY 		 = -KF_RightY;
		KF_RightCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_RightY 		 %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_RightCorrStateEst.aRow[0] = -KF_RightCorrStateEst.aRow[0];
		KF_RightY 		 = -KF_RightY;
		KF_RightModCntr--;
	}
#endif
}

static void KF_SetInitialValues()
{
	uint8_t i = 0u;
	uint8_t j = 0u;
	for(i = 0u; i < KF_SYS_DIMENSION; i++)
	{
		KF_LeftCorrStateEst.aRow[i] = kfCfg->x0->aRow[i];
		KF_RightCorrStateEst.aRow[i] = kfCfg->x0->aRow[i];
	}
	for(i = 0u; i < KF_SYS_DIMENSION; i++)
	{
		for(j = 0u; j < KF_SYS_DIMENSION; j++)
		{
			KF_LeftCorrErrInEst.aRow[i].aCol[j] = kfCfg->P0->aRow[i].aCol[j];
			KF_RightCorrErrInEst.aRow[i].aCol[j] = kfCfg->P0->aRow[i].aCol[j];
		}
	}
}


static StdRtn_t KF_AddMatrices(const KF_I32Mat_t* M_, const KF_I32Mat_t* N_, KF_I32Mat_t* result_, bool subtract_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		for(i = 0u; i < KF_SYS_DIMENSION; i++) //make sure every element in Result is 0!
		{
			for(j = 0u; j < KF_SYS_DIMENSION; j++)
			{
					result_->aRow[i].aCol[j] = 0;
			}
		}
		if(TRUE == subtract_)
		{
			for(i = 0u; i < KF_SYS_DIMENSION; i++)
			{
				for(j = 0u; j < KF_SYS_DIMENSION; j++)
				{
					result_->aRow[i].aCol[j] = M_->aRow[i].aCol[j] - N_->aRow[i].aCol[j];
				}
			}
		}else //add in any other case
		{
			for(i = 0u; i < KF_SYS_DIMENSION; i++)
			{
				for(j = 0u; j < KF_SYS_DIMENSION; j++)
				{
					result_->aRow[i].aCol[j] = M_->aRow[i].aCol[j] + N_->aRow[i].aCol[j];
				}
			}
		}
	}
	return retVal;
}

static StdRtn_t KF_MultMatrices(const KF_I32Mat_t* M_, const KF_I32Mat_t* N_, KF_I32Mat_t* result_, int32_t divider_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	uint8_t k = 0u;

	if( NULL != result_ )
	{
		retVal = ERR_OK;
		for(i = 0u; i < KF_SYS_DIMENSION; i++) //make sure every element in Result is 0!
		{
			for(j = 0u; j < KF_SYS_DIMENSION; j++)
			{
					result_->aRow[i].aCol[j] = 0;
			}
		}

		for(i = 0u; i < KF_SYS_DIMENSION; i++)
		{
			for(j = 0u; j < KF_SYS_DIMENSION; j++)
			{
				for(k = 0u; k < KF_SYS_DIMENSION; k++)
				{
					result_->aRow[i].aCol[j] += (M_->aRow[i].aCol[k] * N_->aRow[k].aCol[j])/(divider_);
				}
			}
		}
	}

	return retVal;
}

static StdRtn_t KF_TransposeMat(const KF_I32Mat_t* M_, KF_I32Mat_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		for(i=0u; i < KF_SYS_DIMENSION; i++)
		{
			for(j = 0; j < KF_SYS_DIMENSION; j++)
			{
				result_->aRow[i].aCol[j] = M_->aRow[j].aCol[i];
			}
		}
	}
	return retVal;
}

static StdRtn_t KF_CalcDeterminant(const KF_I32Mat_t* m_,const KF_I32MatLowDim_t* mlow_, int32_t* result_, int32_t scaleDown_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t tempDet = 0;
	uint8_t i = 0u;
	uint8_t j = 0u;
	bool  calcLowDim = FALSE;
	if (NULL == m_ && NULL != mlow_) // decide if det of lower dim matrix should be calculated
	{
		calcLowDim = TRUE;
	}

	if(NULL != result_){
		retVal = ERR_OK;
		if(TRUE == calcLowDim)
		{
			if(2 == (KF_SYS_DIMENSION-1))
			{
				tempDet = (mlow_->aRow[0].aCol[0]*mlow_->aRow[1].aCol[1])/scaleDown_ - (mlow_->aRow[0].aCol[1]*mlow_->aRow[1].aCol[0])/scaleDown_;
			}
			else if(3 == (KF_SYS_DIMENSION-1))
			{
				for(i = 0u; i < (KF_SYS_DIMENSION-1); i++) //Regel von Sarrus
				{
					tempDet += ((((mlow_->aRow[0].aCol[i]*mlow_->aRow[1].aCol[(i+1)%(KF_SYS_DIMENSION-1)])/scaleDown_)*mlow_->aRow[2].aCol[(i+2)%(KF_SYS_DIMENSION-1)])/scaleDown_) - (((mlow_->aRow[0].aCol[(i+1)%(KF_SYS_DIMENSION-1)]*mlow_->aRow[1].aCol[(i+3)%(KF_SYS_DIMENSION-1)])/scaleDown_)*mlow_->aRow[2].aCol[(i+2)%(KF_SYS_DIMENSION-1)])/scaleDown_;
				}
			}
		}else
		{
			if(2 == KF_SYS_DIMENSION)
			{
				tempDet = (m_->aRow[0].aCol[0]*m_->aRow[1].aCol[1])/scaleDown_ - (m_->aRow[0].aCol[1]*m_->aRow[1].aCol[0])/scaleDown_;
			}
			else if(3 == KF_SYS_DIMENSION)
			{
				for(i = 0u; i < KF_SYS_DIMENSION; i++) //Regel von Sarrus
				{
					tempDet += (((m_->aRow[0].aCol[i]*m_->aRow[1].aCol[(i+1)%KF_SYS_DIMENSION])/scaleDown_)*m_->aRow[2].aCol[(i+2)%KF_SYS_DIMENSION]/scaleDown_) - (((m_->aRow[0].aCol[(i+1)%KF_SYS_DIMENSION]*m_->aRow[1].aCol[(i+3)%KF_SYS_DIMENSION])/scaleDown_)*m_->aRow[2].aCol[(i+2)%KF_SYS_DIMENSION])/scaleDown_;
				}
			}
		}

	}
	*result_  = tempDet;
	return retVal;
}

static StdRtn_t KF_CalcMinor(const KF_I32Mat_t* m_, const uint8_t row_, const uint8_t col_, KF_I32MatLowDim_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	uint8_t k = 0u;
	uint8_t l = 0u;

	if(NULL != result_)
	{
		retVal = ERR_OK;

				for(k = 0; k < KF_SYS_DIMENSION; k++)
				{
					if(k == row_) continue;
					for(l = 0; l < KF_SYS_DIMENSION; l++)
					{
						if(l == col_) continue;
						result_->aRow[i].aCol[j] = m_->aRow[k].aCol[l];
						j++;
						if(j == (KF_SYS_DIMENSION-1)) j = 0;
					}
					i++;
				}
	}
	return retVal;
}

static StdRtn_t KF_InvertMatrix(const KF_I32Mat_t* m_, const int32_t det_m_, KF_I32Mat_t* result_, int32_t scaleDown_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t tempDet = 0;
	uint8_t i = 0u;
	uint8_t j = 0u;
	int8_t negate = 1;
	KF_I32MatLowDim_t minor = {0};

	if(NULL != result_)
	{
		retVal = ERR_VALUE;
		for(i = 0u; i < KF_SYS_DIMENSION; i++) //make sure every element in Result is 0!
		{
			for(j = 0u; j < KF_SYS_DIMENSION; j++)
			{
					result_->aRow[i].aCol[j] = 0;
			}
		}

		if(0 != det_m_)
		{
			retVal = ERR_OK;
			if(2 == KF_SYS_DIMENSION) //2x2 matrices
			{
				result_->aRow[0].aCol[0] = (m_->aRow[KF_SYS_DIMENSION-1].aCol[KF_SYS_DIMENSION-1])/det_m_;
				result_->aRow[0].aCol[1] = (-m_->aRow[0].aCol[1])/det_m_;
				result_->aRow[1].aCol[0] = (-m_->aRow[1].aCol[0])/det_m_;
				result_->aRow[1].aCol[1] = (m_->aRow[0].aCol[0])/det_m_;
			}
			else //3x3 matrices
			{
				for(i = 0u; i < KF_SYS_DIMENSION; i++) //i-te zeile, j-te spalte
				{
					for(j = 0u; j < KF_SYS_DIMENSION; j++)
					{
						if(0 != (i+j)%2)
						{
							negate = -1;
						}
						else
						{
							negate = 1;
						}
						KF_CalcMinor(m_, j, i, &minor);										//minor is the lower dim matrix constructed by ignoring row j and column i
						KF_CalcDeterminant(NULL, &minor, &tempDet, scaleDown_);							//tempDet is the Cofactor for i and j
						result_->aRow[i].aCol[j] = (negate*(tempDet))/det_m_;   //already transposed!
					}
				}
			}
		}
	}
	return retVal;
}


static StdRtn_t KF_MultRowVecMat(const KF_I32RowVec_t* v_, const KF_I32Mat_t* M_, KF_I32RowVec_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		for (i = 0u; i < KF_SYS_DIMENSION; i++) // make sure every element in result is 0!
		{
			result_->aCol[i] = 0;
		}

		for(i = 0u; i < KF_SYS_DIMENSION; i++)
		{
			for (j = 0u; j < KF_SYS_DIMENSION; j++)
			{
				result_->aCol[i] += v_->aCol[j] * M_->aRow[j].aCol[i];
			}
		}
	}
	return retVal;
}

static StdRtn_t KF_MultMatColVec(const KF_I32Mat_t* M_, const KF_I32ColVec_t* v_, KF_I32ColVec_t* result_, int32_t divider_) //2x2
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;

	if( NULL != result_ )
	{
		retVal = ERR_OK;
		for(i = 0u; i < KF_SYS_DIMENSION; i++) //Make sure every element in result is 0;
		{
			result_->aRow[i] = 0;
		}

		for(i = 0u; i < KF_SYS_DIMENSION; i++)
		{
			for(j = 0u; j < KF_SYS_DIMENSION; j++)
			{
				result_->aRow[i] += (M_->aRow[i].aCol[j] * v_->aRow[j])/(divider_);
			}
		}

	}

	return retVal;
}


static StdRtn_t KF_MultRowVecColVec(const KF_I32RowVec_t* v_, const KF_I32ColVec_t* w_, int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	if (NULL != result_)
	{
		retVal = ERR_OK;
		(*result_) = 0; //make sure result is 0!

		for(i = 0u; i < KF_SYS_DIMENSION; i++)
		{
			(*result_) += v_->aCol[i] * w_->aRow[i];
		}
	}
	return retVal;
}

static StdRtn_t KF_TransposeRowVec(const KF_I32RowVec_t* v_, KF_I32ColVec_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		for(i = 0u; i < KF_SYS_DIMENSION; i++)
		{
			result_->aRow[i] = v_->aCol[i];
		}
	}
	return retVal;
}

static StdRtn_t KF_MultColVecFactor(const KF_I32ColVec_t* v_, const int32_t factor_, KF_I32ColVec_t* result_, bool divide_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		if(TRUE == divide_)
		{
			for(i = 0u; i < KF_SYS_DIMENSION; i++)
			{
				result_->aRow[i] = (v_->aRow[i])/(factor_);
			}
		}else
		{
			for(i = 0u; i < KF_SYS_DIMENSION; i++)
			{
				result_->aRow[i] = ((factor_)*v_->aRow[i]);
			}
		}
	}
	return retVal;
}

static StdRtn_t KF_AddColVecs(const KF_I32ColVec_t* v_,const KF_I32ColVec_t* w_, KF_I32ColVec_t* result_, bool subtract_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		if(TRUE == subtract_)
		{
			for(i = 0u; i < KF_SYS_DIMENSION; i++)
			{
				result_->aRow[i] = v_->aRow[i] - w_->aRow[i];
			}
		}else //add in any other case
		{
			for(i = 0u; i < KF_SYS_DIMENSION; i++)
			{
				result_->aRow[i] = v_->aRow[i] + w_->aRow[i];
			}
		}
	}
	return retVal;
}

static StdRtn_t KF_MultColVecRowVec(const KF_I32ColVec_t* v_, const KF_I32RowVec_t* w_, KF_I32Mat_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		for(i = 0u; i < KF_SYS_DIMENSION; i++)
		{
			for(j = 0u; j < KF_SYS_DIMENSION; j++)
			{
					result_->aRow[i].aCol[j] = v_->aRow[i] * w_->aCol[j];
			}
		}
	}
	return retVal;
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
int32_t KF_GetSpeed(bool isLeft_)
{
	int32_t speed = 0;
	if(TRUE == isLeft_)
	{
		speed = KF_LeftSpeed;
	}
	else
	{
		speed = KF_RightSpeed;
	}
	return speed;
}

int32_t KF_GetPosition(bool isLeft_)
{
	int32_t position = 0;
	if(TRUE == isLeft_)
	{
		position = KF_LeftPos;
	}else
	{
		position = KF_LeftPos;
	}
	return position;
}

void KF_Init()
{
	StdRtn_t retVal = ERR_OK;

	kfCfg = GetKFCfg();
	if(NULL == kfCfg)
	{
		for(;;){} //error case
	}
	retVal |= KF_TransposeMat(kfCfg->A, &KF_AT);

#if KF_USE_MEASUREMENT_MATRIX
	retVal |= KF_TransposeMat(kfCfg->C, &KF_CT);
#else
	retVal |= KF_TransposeRowVec(kfCfg->cT, &KF_c);
#endif
	if(ERR_OK != retVal)
	{
		for(;;){}
	}
	KF_SetInitialValues();

}

void KF_Main()
{
	StdRtn_t retVal = ERR_OK;

	KF_I32ColVec_t Left_P_times_c	    	= {0};
	KF_I32ColVec_t Left_B_times_u			= {0};
	KF_I32ColVec_t Left_K_times_Residuum 	= {0};
	KF_I32ColVec_t LeftTempColVec 			= {0};
	KF_I32Mat_t Left_Ident_minus_K_times_cT = {0};
	KF_I32Mat_t LeftTempMat  = {0};
	KF_I32Mat_t LeftTempMat2 = {0};
	KF_I32Mat_t LeftTempMat3 = {0};
	int32_t LeftTempResult   = 0;


	KF_I32ColVec_t Right_P_times_c	     	 = {0};
	KF_I32ColVec_t Right_B_times_u			= {0};
	KF_I32ColVec_t Right_K_times_Residuum	 = {0};
	KF_I32ColVec_t RightTempColVec 		  	 = {0};
	KF_I32Mat_t Right_Ident_minus_K_times_cT = {0};
	KF_I32Mat_t RightTempMat  = {0};
	KF_I32Mat_t RightTempMat2 = {0};
	KF_I32Mat_t RightTempMat3 = {0};
	int32_t RightTempResult   =  0;

	KF_UpdateModuloCounter();
	KF_UpdateInput();

	/* Time Update / "predictor" */
		//x_k_hat	KF_PredStateEst scaled to KF_SCALE_X!
		retVal |= KF_MultMatColVec(kfCfg->A, &KF_LeftCorrStateEst, &LeftTempColVec, (int32_t)KF_SCALE_A);
		retVal |= KF_MultMatColVec(kfCfg->B, &KF_LeftU, &Left_B_times_u, (int32_t)1);
		retVal |= KF_AddColVecs(&LeftTempColVec, &Left_B_times_u, &KF_LeftPredStateEst, FALSE);

		retVal |= KF_MultMatColVec(kfCfg->A, &KF_RightCorrStateEst, &KF_RightPredStateEst, (int32_t)KF_SCALE_A);
		retVal |= KF_MultMatColVec(kfCfg->B, &KF_RightU, &Right_B_times_u, (int32_t)1);
		retVal |= KF_AddColVecs(&RightTempColVec, &Right_B_times_u, &KF_RightPredStateEst, FALSE);

		//P_k		KF_PredErrorInEst scaled to KF_SCALE_ERR!
		retVal |= KF_MultMatrices(kfCfg->A, &KF_LeftCorrErrInEst, &LeftTempMat, (int32_t)KF_SCALE_A);
		retVal |= KF_MultMatrices(&LeftTempMat, &KF_AT, &LeftTempMat2, (int32_t)KF_SCALE_A);
		retVal |= KF_AddMatrices(&LeftTempMat2, kfCfg->Q, &KF_LeftPredErrInEst, FALSE);

		retVal |= KF_MultMatrices(kfCfg->A, &KF_RightCorrErrInEst, &RightTempMat, (int32_t)KF_SCALE_A);
		retVal |= KF_MultMatrices(&RightTempMat, &KF_AT, &RightTempMat2, (int32_t)KF_SCALE_A);
		retVal |= KF_AddMatrices(&RightTempMat2, kfCfg->Q, &KF_RightPredErrInEst, FALSE);

	/* Measurement Update / "corrector" */
		//KalmanGain	KF_KalmanGain scaled to KF_SCALE_KALMANGAIN
			#if KF_USE_MEASUREMENT_MATRIX
				KF_I32Mat_t Left_P_times_CT  = {0};
				KF_I32Mat_t Right_P_times_CT = {0};
				KF_I32Mat_t Left_C_times_P   = {0};
				KF_I32Mat_t Right_C_times_P  = {0};
				uint8_t i = 0u;
				uint8_t j = 0u;

				//left
				retVal |= KF_MultMatrices(&KF_LeftPredErrInEst, &KF_CT, &Left_P_times_CT, (int32_t)1);
				retVal |= KF_MultMatrices(&Left_P_times_CT, &KF_Ident, &LeftTempMat3, (int32_t)KF_SCALE_ERROR);
				retVal |= KF_MultMatrices(kfCfg->C, &KF_LeftPredErrInEst, &Left_C_times_P, (int32_t)1);
				retVal |= KF_MultMatrices(&Left_C_times_P, &KF_CT, &LeftTempMat, (int32_t)1);
				retVal |= KF_AddMatrices(&LeftTempMat, kfCfg->R, &KF_LeftDenominator, FALSE);
				retVal |= KF_MultMatrices(&KF_LeftDenominator, &KF_Ident, &LeftTempMat, (int32_t)KF_SCALE_ERROR);
				retVal |= KF_CalcDeterminant(&LeftTempMat, NULL, &LeftTempResult, (int32_t)1);
				LeftTempResult/=KF_SCALE_DET;
				retVal |= KF_InvertMatrix(&KF_LeftDenominator, LeftTempResult, &LeftTempMat2, (int32_t)1);
				retVal |= KF_MultMatrices(&LeftTempMat3, &LeftTempMat2, &LeftTempMat, (int32_t)KF_SCALE_DET);
				retVal |= KF_MultMatrices(&LeftTempMat, kfCfg->I, &KF_LeftKalmanGain, (int32_t)KF_SCALE_ERROR);

				//right
				retVal |= KF_MultMatrices(&KF_RightPredErrInEst, &KF_CT, &Right_P_times_CT, (int32_t)1);
				retVal |= KF_MultMatrices(&Right_P_times_CT, &KF_Ident, &RightTempMat3, (int32_t)KF_SCALE_ERROR);
				retVal |= KF_MultMatrices(kfCfg->C, &KF_RightPredErrInEst, &Right_C_times_P, (int32_t)1);
				retVal |= KF_MultMatrices(&Right_C_times_P, &KF_CT, &RightTempMat, (int32_t)1);
				retVal |= KF_AddMatrices(&RightTempMat, kfCfg->R, &KF_RightDenominator, FALSE);
				retVal |= KF_MultMatrices(&KF_RightDenominator, &KF_Ident, &RightTempMat, (int32_t)KF_SCALE_ERROR);
				retVal |= KF_CalcDeterminant(&RightTempMat, NULL, &RightTempResult, (int32_t)1);
				RightTempResult/=KF_SCALE_DET;
				retVal |= KF_InvertMatrix(&KF_RightDenominator, RightTempResult, &RightTempMat2, (int32_t)1);
				retVal |= KF_MultMatrices(&RightTempMat3, &RightTempMat2, &RightTempMat, (int32_t)KF_SCALE_DET);
				retVal |= KF_MultMatrices(&RightTempMat, kfCfg->I, &KF_RightKalmanGain, (int32_t)KF_SCALE_ERROR);

			#else
				//left
				retVal |= KF_MultMatColVec(&KF_LeftPredErrInEst, &KF_c, &Left_P_times_c, 1); 		//numerator
				retVal |= KF_MultRowVecColVec(kfCfg->cT, &Left_P_times_c, &LeftTempResult);		//denominator
				KF_LeftDenominator = (LeftTempResult + (*kfCfg->r));  								//denominator
				Left_P_times_c.aRow[0] *= (int32_t)KF_SCALE_KALMANGAIN; //scale numerator
				Left_P_times_c.aRow[1] *= (int32_t)KF_SCALE_KALMANGAIN;
				retVal |= KF_MultColVecFactor(&Left_P_times_c, KF_LeftDenominator, &KF_LeftKalmanGain, TRUE);

				//right
				retVal |= KF_MultMatColVec(&KF_RightPredErrInEst, &KF_c, &Right_P_times_c, 1); 		//numerator
				retVal |= KF_MultRowVecColVec(kfCfg->cT, &Right_P_times_c, &RightTempResult);		//denominator
				KF_RightDenominator = (RightTempResult + (*kfCfg->r));  								//denominator
				Right_P_times_c.aRow[0] *= (int32_t)KF_SCALE_KALMANGAIN; //scale numerator
				Right_P_times_c.aRow[1] *= (int32_t)KF_SCALE_KALMANGAIN;
				retVal |= KF_MultColVecFactor(&Right_P_times_c, KF_RightDenominator, &KF_RightKalmanGain, TRUE);
			#endif

		//x_k_hat
#if KF_USE_MEASUREMENT_MATRIX
		//left
		retVal |= KF_MultMatColVec(kfCfg->C, &KF_LeftPredStateEst, &LeftTempColVec, (int32_t)1);
		retVal |= KF_AddColVecs(&KF_LeftY, &LeftTempColVec, &KF_LeftResiduum, TRUE);
		retVal |= KF_MultMatColVec(&KF_LeftKalmanGain, &KF_LeftResiduum, &Left_K_times_Residuum, (int32_t)KF_SCALE_KALMANGAIN);

		//right
		retVal |= KF_MultMatColVec(kfCfg->C, &KF_RightPredStateEst, &RightTempColVec, (int32_t)1);
		retVal |= KF_AddColVecs(&KF_RightY, &RightTempColVec, &KF_RightResiduum, TRUE);
		retVal |= KF_MultMatColVec(&KF_RightKalmanGain, &KF_RightResiduum, &Right_K_times_Residuum, (int32_t)KF_SCALE_KALMANGAIN);
#else
		KF_LeftResiduum = KF_LeftY - KF_LeftCorrStateEst.aRow[0];
		retVal |= KF_MultColVecFactor(&KF_LeftKalmanGain, KF_LeftResiduum, &LeftTempColVec, FALSE);
		retVal |= KF_MultColVecFactor(&LeftTempColVec, (int32_t)KF_SCALE_KALMANGAIN, &Left_K_times_Residuum, TRUE);

		KF_RightResiduum = KF_RightY - KF_RightCorrStateEst.aRow[0];
		retVal |= KF_MultColVecFactor(&KF_RightKalmanGain, KF_RightResiduum, &RightTempColVec, FALSE);
		retVal |= KF_MultColVecFactor(&RightTempColVec, (int32_t)KF_SCALE_KALMANGAIN, &Right_K_times_Residuum, TRUE);
#endif
		//left
		retVal |= KF_AddColVecs(&KF_LeftPredStateEst, &Left_K_times_Residuum, &KF_LeftCorrStateEst, FALSE);
		//right
		retVal |= KF_AddColVecs(&KF_RightPredStateEst, &Right_K_times_Residuum, &KF_RightCorrStateEst, FALSE);


		KF_UpdateMeasurements();

		//P_k
#if KF_USE_MEASUREMENT_MATRIX
		retVal |= KF_MultMatrices(&KF_LeftKalmanGain, kfCfg->C, &LeftTempMat, (int32_t)1);

		retVal |= KF_MultMatrices(&KF_RightKalmanGain, kfCfg->C, &RightTempMat, (int32_t)1);
#else
		retVal |= KF_MultColVecRowVec(&KF_LeftKalmanGain, kfCfg->cT, &LeftTempMat);

		retVal |= KF_MultColVecRowVec(&KF_RightKalmanGain, kfCfg->cT, &RightTempMat);
#endif
		retVal |= KF_AddMatrices(kfCfg->I, &LeftTempMat, &LeftTempMat2, TRUE);
		retVal |= KF_MultMatrices(&LeftTempMat2, &KF_LeftPredErrInEst, &KF_LeftCorrErrInEst, (int32_t)KF_SCALE_KALMANGAIN);

		retVal |= KF_AddMatrices(kfCfg->I, &RightTempMat, &RightTempMat2, TRUE);
		retVal |= KF_MultMatrices(&RightTempMat2, &KF_RightPredErrInEst, &KF_RightCorrErrInEst, (int32_t)KF_SCALE_KALMANGAIN);

	if(ERR_OK != retVal)
	{
		for(;;){} //error case
	}

	KF_UpdateUnscaledVals();

}

#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
