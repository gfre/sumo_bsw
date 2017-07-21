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
static StdRtn_t KF_InvertMatrix(const KF_I32Mat_t* m_, const int32_t factor_, KF_I32Mat_t* result_);
static StdRtn_t KF_Minor(const KF_I32Mat_t* m_, const uint8_t* row_, const uint8_t* col_, KF_I32MatLowDim_t* result_);
static StdRtn_t KF_Determinant(const KF_I32Mat_t* m_, const KF_I32MatLowDim_t* mlow_, int32_t* result_);

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

static KF_I32Mat_t 	  KF_SystemMatrixTransposed = {0};



static KF_I32Mat_t 	  KF_LeftCorrErrorInEst  = {0};
static KF_I32Mat_t 	  KF_RightCorrErrorInEst = {0};
static KF_I32Mat_t	  KF_LeftPredErrorInEst  = {0};
static KF_I32Mat_t	  KF_RightPredErrorInEst = {0};

static KF_I32ColVec_t KF_LeftCorrStateEst 	 = {0};
static KF_I32ColVec_t KF_RightCorrStateEst 	 = {0};
static KF_I32ColVec_t KF_LeftPredStateEst 	 = {0};
static KF_I32ColVec_t KF_RightPredStateEst 	 = {0};

static int32_t 		  KF_LeftUnscaledEstVel  = 0;
static int32_t 		  KF_RightUnscaledEstVel = 0;
static int32_t 		  KF_LeftUnscaledEstPos  = 0;
static int32_t 		  KF_RightUnscaledEstPos = 0;

#if KF_USE_MEASUREMENT_MATRIX
	static KF_I32Mat_t KF_MeasurementMatrixTransp = {0};

	static KF_I32Mat_t KF_LeftKalmanGain = {0};
	static KF_I32Mat_t KF_RightKalmanGain = {0};
	static KF_I32ColVec_t KF_LeftResiduum = {0};
	static KF_I32ColVec_t KF_RightResiduum = {0};
	static KF_I32ColVec_t KF_LeftMeasurements = {0};
	static KF_I32ColVec_t KF_RightMeasurements = {0};
#else

	static KF_I32ColVec_t KF_MeasurementVector 		= {0};

	static KF_I32ColVec_t KF_LeftKalmanGain 	 = {0};
	static KF_I32ColVec_t KF_RightKalmanGain 	 = {0};
	static int32_t KF_LeftResiduum 		  = 0; //can be local?
	static int32_t KF_RightResiduum 	  = 0; //can be local?
	static int32_t KF_RightPosMeasurement = 0;
	static int32_t KF_LeftPosMeasurement  = 0;
	static int32_t KF_RightDenominator 	  = 0; //can be local?
	static int32_t KF_LeftDenominator 	  = 0; //can be local?

#endif

static int16_t KF_LeftModuloCntr 	  = 0;
static int16_t KF_RightModuloCntr 	  = 0;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void KF_UpdateMeasurements()
{
		KF_LeftPosMeasurement  += ((Q4CLeft_GetPos()*KF_SCALE_X)-(KF_LeftPosMeasurement + (KF_LeftModuloCntr*(KF_MAX_POS_VAL/KF_SCALE_A))));

		KF_RightPosMeasurement += ((Q4CRight_GetPos()*KF_SCALE_X)-(KF_RightPosMeasurement + (KF_RightModuloCntr*(KF_MAX_POS_VAL/KF_SCALE_A))));
}

static void KF_UpdateUnscaledVals()
{
	KF_LeftUnscaledEstPos = (KF_LeftModuloCntr*((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) + KF_LeftCorrStateEst.aRow[0])/((int32_t)KF_SCALE_X);
	KF_LeftUnscaledEstVel = KF_LeftCorrStateEst.aRow[1]/(int32_t)KF_SCALE_X;

	KF_RightUnscaledEstPos = (KF_RightModuloCntr*((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) + KF_RightCorrStateEst.aRow[0])/((int32_t)KF_SCALE_X);
	KF_RightUnscaledEstVel = KF_RightCorrStateEst.aRow[1]/(int32_t)KF_SCALE_X;
}

static void KF_UpdateModuloCounter()
{
	if((KF_LeftCorrStateEst.aRow[0] >= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_LeftPosMeasurement) >= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
	{
		KF_LeftCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_LeftPosMeasurement 		%= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_LeftModuloCntr++;
	}else if((KF_LeftCorrStateEst.aRow[0] <= -(int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_LeftPosMeasurement) <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
	{
		KF_LeftCorrStateEst.aRow[0] = -KF_LeftCorrStateEst.aRow[0];
		KF_LeftPosMeasurement 		= -KF_LeftPosMeasurement;
		KF_LeftCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_LeftPosMeasurement 		%= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_LeftCorrStateEst.aRow[0] = -KF_LeftCorrStateEst.aRow[0];
		KF_LeftPosMeasurement 		= -KF_LeftPosMeasurement;
		KF_LeftModuloCntr--;
	}

	if((KF_RightCorrStateEst.aRow[0] >= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_RightPosMeasurement) >= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
	{
		KF_RightCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_RightPosMeasurement 		 %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_RightModuloCntr++;
	}else if((KF_RightCorrStateEst.aRow[0] <= -(int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((KF_RightPosMeasurement) <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
	{
		KF_RightCorrStateEst.aRow[0] = -KF_RightCorrStateEst.aRow[0];
		KF_RightPosMeasurement 		 = -KF_RightPosMeasurement;
		KF_RightCorrStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_RightPosMeasurement 		 %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
		KF_RightCorrStateEst.aRow[0] = -KF_RightCorrStateEst.aRow[0];
		KF_RightPosMeasurement 		 = -KF_RightPosMeasurement;
		KF_RightModuloCntr--;
	}
}

static void KF_SetInitialValues()

{
	uint8_t i = 0u;
	uint8_t j = 0u;
	for(i = 0u; i < KF_SYS_DIMENSION; i++)
	{
		KF_LeftCorrStateEst.aRow[i] = kfCfg->StateInitialEstimate->aRow[i];
		KF_RightCorrStateEst.aRow[i] = kfCfg->StateInitialEstimate->aRow[i];
	}
	for(i = 0u; i < KF_SYS_DIMENSION; i++)
	{
		for(j = 0u; j < KF_SYS_DIMENSION; j++)
		{
			KF_LeftCorrErrorInEst.aRow[i].aCol[j] = kfCfg->InitialErrorInEstimate->aRow[i].aCol[j];
			KF_RightCorrErrorInEst.aRow[i].aCol[j] = kfCfg->InitialErrorInEstimate->aRow[i].aCol[j];
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

static StdRtn_t KF_Determinant(const KF_I32Mat_t* m_,const KF_I32MatLowDim_t* mlow_, int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	bool  calcLowDim = FALSE;
	if (NULL == m_ && NULL != mlow_) // decide if det of lower dim matrix should be calculated
	{
		calcLowDim = TRUE;
	}

	if(NULL != result_){
		retVal = ERR_OK;
		*result_ = 0;
		if(TRUE == calcLowDim)
		{
			if(2 == (KF_SYS_DIMENSION-1))
			{
				*result_ = mlow_->aRow[0].aCol[0]*mlow_->aRow[1].aCol[1] - mlow_->aRow[0].aCol[1]*mlow_->aRow[1].aCol[0];
			}
			else if(3 == (KF_SYS_DIMENSION-1))
			{
				for(i = 0u; i < (KF_SYS_DIMENSION-1); i++) //Regel von Sarrus
				{
					result_ += (mlow_->aRow[0].aCol[i]*(mlow_->aRow[1].aCol[(i+1)%(KF_SYS_DIMENSION-1)]*mlow_->aRow[2].aCol[(i+2)%(KF_SYS_DIMENSION-1)] - mlow_->aRow[0].aCol[(i+1)%(KF_SYS_DIMENSION-1)]*mlow_->aRow[1].aCol[(i+3)%(KF_SYS_DIMENSION-1)]*mlow_->aRow[2].aCol[(i+2)%(KF_SYS_DIMENSION-1)]));
				}
			}
		}else
		{
			if(2 == KF_SYS_DIMENSION)
			{
				*result_ = m_->aRow[0].aCol[0]*m_->aRow[1].aCol[1] - m_->aRow[0].aCol[1]*m_->aRow[1].aCol[0];
			}
			else if(3 == KF_SYS_DIMENSION)
			{
				for(i = 0u; i < KF_SYS_DIMENSION; i++) //Regel von Sarrus
				{
					result_ += (m_->aRow[0].aCol[i]*(m_->aRow[1].aCol[(i+1)%KF_SYS_DIMENSION]*m_->aRow[2].aCol[(i+2)%KF_SYS_DIMENSION] - m_->aRow[0].aCol[(i+1)%KF_SYS_DIMENSION]*m_->aRow[1].aCol[(i+3)%KF_SYS_DIMENSION]*m_->aRow[2].aCol[(i+2)%KF_SYS_DIMENSION]));
				}
			}
		}

	}
	return retVal;
}

static StdRtn_t KF_Minor(const KF_I32Mat_t* m_, const uint8_t* row_, const uint8_t* col_, KF_I32MatLowDim_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	uint8_t k = 0u;
	uint8_t l = 0u;

	if(NULL != result_)
	{
		retVal = ERR_OK;
		for(i = 0u; i < KF_SYS_DIMENSION-1; i++)
		{
			for(j = 0u; j < KF_SYS_DIMENSION-1; j++)
			{
				for(k = 0; k < KF_SYS_DIMENSION; k++)
				{
					if(k == *row_) continue;
					for(l = 0; l < KF_SYS_DIMENSION; l++)
					{
						if(l == *col_) continue;
						result_->aRow[i].aCol[j] = m_->aRow[k].aCol[l];
					}
				}
			}
		}
	}
	return retVal;
}

static StdRtn_t KF_InvertMatrix(const KF_I32Mat_t* m_, const int32_t factor_, KF_I32Mat_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t det_m = 0;
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

		KF_Determinant(m_, NULL, &det_m);
		if(0 != det_m)
		{
			retVal = ERR_OK;
			if(2 == KF_SYS_DIMENSION)
			{
				result_->aRow[0].aCol[0] = (factor_*m_->aRow[KF_SYS_DIMENSION-1].aCol[KF_SYS_DIMENSION-1])/det_m;
				result_->aRow[0].aCol[1] = (-factor_*m_->aRow[0].aCol[1])/det_m;
				result_->aRow[1].aCol[0] = (-factor_*m_->aRow[1].aCol[0])/det_m;
				result_->aRow[1].aCol[1] = (factor_*m_->aRow[0].aCol[0])/det_m;
			}
			else
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
						KF_Minor(m_, &j, &i, &minor);									//minor is the lower dim matrix constructed by ignoring row j and column i
						KF_Determinant(NULL, &minor, &tempDet);							//tempDet is the Cofactor for i and j
						result_->aRow[j].aCol[i] = (negate*(factor_*tempDet))/det_m;    //already transposed!
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
		speed = KF_LeftUnscaledEstVel;
	}
	else
	{
		speed = KF_RightUnscaledEstVel;
	}
	return speed;
}

int32_t KF_GetPosition(bool isLeft_)
{
	int32_t position = 0;
	if(TRUE == isLeft_)
	{
		position = KF_LeftUnscaledEstPos;
	}else
	{
		position = KF_RightUnscaledEstPos;
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
	retVal |= KF_TransposeMat(kfCfg->SystemMatrix, &KF_SystemMatrixTransposed);
	retVal |= KF_TransposeRowVec(kfCfg->MeasurementVectorTransposed, &KF_MeasurementVector);
	if(ERR_OK != retVal)
	{
		for(;;){}
	}
	KF_SetInitialValues();

	KF_I32Mat_t testInvert;

	KF_InvertMatrix(kfCfg->SystemMatrix, (int32)1000000, &testInvert);
}

void KF_Main()
{
	StdRtn_t retVal = ERR_OK;

	KF_I32ColVec_t Left_P_times_c	    = {0};
	KF_I32ColVec_t Left_K_times_Residuum = {0};
	KF_I32ColVec_t LeftTempColVec 		= {0};
	KF_I32Mat_t Left_Ident_minus_K_times_cT = {0};
	KF_I32Mat_t LeftTempMat  = {0};
	KF_I32Mat_t LeftTempMat2 = {0};
	KF_I32Mat_t LeftTempMat3 = {0};
	int32_t LeftTempResult = 0;


	KF_I32ColVec_t Right_P_times_c	    = {0};
	KF_I32ColVec_t Right_K_times_Residuum = {0};
	KF_I32ColVec_t RightTempColVec 		= {0};
	KF_I32Mat_t Right_Ident_minus_K_times_cT = {0};
	KF_I32Mat_t RightTempMat  = {0};
	KF_I32Mat_t RightTempMat2 = {0};
	KF_I32Mat_t RightTempMat3 = {0};
	int32_t RightTempResult = 0;

	KF_UpdateModuloCounter();
#if KF_USE_C_MATRIX





#else
	/* Time Update / "predictor" */
		//x_k_hat
		retVal |= KF_MultMatColVec(kfCfg->SystemMatrix, &KF_LeftCorrStateEst, &KF_LeftPredStateEst, (int32_t)KF_SCALE_A); //scale x

		retVal |= KF_MultMatColVec(kfCfg->SystemMatrix, &KF_RightCorrStateEst, &KF_RightPredStateEst, (int32_t)KF_SCALE_A); //scale x

		//P_k
		retVal |= KF_MultMatrices(kfCfg->SystemMatrix, &KF_LeftCorrErrorInEst, &LeftTempMat, (int32_t)KF_SCALE_A);
		retVal |= KF_MultMatrices(&LeftTempMat, &KF_SystemMatrixTransposed, &LeftTempMat2, (int32_t)KF_SCALE_A);
		retVal |= KF_AddMatrices(&LeftTempMat2, kfCfg->ProcessNoiseCov, &KF_LeftPredErrorInEst, FALSE); //ERROR IN ESTIMATE

		retVal |= KF_MultMatrices(kfCfg->SystemMatrix, &KF_RightCorrErrorInEst, &RightTempMat, (int32_t)KF_SCALE_A);
		retVal |= KF_MultMatrices(&RightTempMat, &KF_SystemMatrixTransposed, &RightTempMat2, (int32_t)KF_SCALE_A);
		retVal |= KF_AddMatrices(&RightTempMat2, kfCfg->ProcessNoiseCov, &KF_RightPredErrorInEst, FALSE); //ERROR IN ESTIMATE

	/* Measurement Update / "corrector" */
		//KalmanGain
		retVal |= KF_MultMatColVec(&KF_LeftPredErrorInEst, &KF_MeasurementVector, &Left_P_times_c, 1); 		//numerator
		retVal |= KF_MultRowVecColVec(kfCfg->MeasurementVectorTransposed, &Left_P_times_c, &LeftTempResult);		//denominator
		KF_LeftDenominator = (LeftTempResult + (*kfCfg->MeasurementNoiseCov));  								//denominator

		retVal |= KF_MultMatColVec(&KF_RightPredErrorInEst, &KF_MeasurementVector, &Right_P_times_c, 1); 		//numerator
		retVal |= KF_MultRowVecColVec(kfCfg->MeasurementVectorTransposed, &Right_P_times_c, &RightTempResult);		//denominator
		KF_RightDenominator = (RightTempResult + (*kfCfg->MeasurementNoiseCov));  								//denominator

		Left_P_times_c.aRow[0] *= (int32_t)KF_SCALE_KALMANGAIN; //scale numerator
		Left_P_times_c.aRow[1] *= (int32_t)KF_SCALE_KALMANGAIN;

		Right_P_times_c.aRow[0] *= (int32_t)KF_SCALE_KALMANGAIN; //scale numerator
		Right_P_times_c.aRow[1] *= (int32_t)KF_SCALE_KALMANGAIN;

		retVal |= KF_MultColVecFactor(&Left_P_times_c, KF_LeftDenominator, &KF_LeftKalmanGain, TRUE);

		retVal |= KF_MultColVecFactor(&Right_P_times_c, KF_RightDenominator, &KF_RightKalmanGain, TRUE);


		//x_k_hat
		KF_LeftResiduum = KF_LeftPosMeasurement - KF_LeftCorrStateEst.aRow[0];
		retVal |= KF_MultColVecFactor(&KF_LeftKalmanGain, KF_LeftResiduum, &Left_K_times_Residuum, FALSE);
		retVal |= KF_MultColVecFactor(&Left_K_times_Residuum, (int32_t)KF_SCALE_KALMANGAIN, &LeftTempColVec, TRUE);
		retVal |= KF_AddColVecs(&KF_LeftPredStateEst, &LeftTempColVec, &KF_LeftCorrStateEst, FALSE);

		KF_RightResiduum = KF_RightPosMeasurement - KF_RightCorrStateEst.aRow[0];
		retVal |= KF_MultColVecFactor(&KF_RightKalmanGain, KF_RightResiduum, &Right_K_times_Residuum, FALSE);
		retVal |= KF_MultColVecFactor(&Right_K_times_Residuum, (int32_t)KF_SCALE_KALMANGAIN, &RightTempColVec, TRUE);
		retVal |= KF_AddColVecs(&KF_RightPredStateEst, &RightTempColVec, &KF_RightCorrStateEst, FALSE);


		KF_UpdateMeasurements();

		//P_k
		retVal |= KF_MultColVecRowVec(&KF_LeftKalmanGain, kfCfg->MeasurementVectorTransposed, &LeftTempMat3);
		retVal |= KF_AddMatrices(kfCfg->IdentityMatrix, &LeftTempMat3, &Left_Ident_minus_K_times_cT, TRUE);
		retVal |= KF_MultMatrices(&Left_Ident_minus_K_times_cT, &KF_LeftPredErrorInEst, &KF_LeftCorrErrorInEst, (int32_t)KF_SCALE_KALMANGAIN);

		retVal |= KF_MultColVecRowVec(&KF_RightKalmanGain, kfCfg->MeasurementVectorTransposed, &RightTempMat3);
		retVal |= KF_AddMatrices(kfCfg->IdentityMatrix, &RightTempMat3, &Right_Ident_minus_K_times_cT, TRUE);
		retVal |= KF_MultMatrices(&Right_Ident_minus_K_times_cT, &KF_RightPredErrorInEst, &KF_RightCorrErrorInEst, (int32_t)KF_SCALE_KALMANGAIN);
#endif
	if(ERR_OK != retVal)
	{
		for(;;){} //error case
	}

	KF_UpdateUnscaledVals();

}

#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
