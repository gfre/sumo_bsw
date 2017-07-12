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
#include "Q4CLeft.h"
#include "tacho_api.h"


/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void KF_UpdateMeasurement(void);
static void KF_UpdateUnscaledVals(void);
static void KF_UpdateModuloCounter(void);
static void KF_SetInitialValues(void);

//Matrix operations
static StdRtn_t KF_AddMatrices(const KF_I32Mat_t* m_, const KF_I32Mat_t* n_, KF_I32Mat_t* result_, bool subtract_);
static StdRtn_t KF_MultMatrices(const KF_I32Mat_t* m_, const KF_I32Mat_t* v_, KF_I32Mat_t* result_, int32_t divider_);
static StdRtn_t KF_TransposeMat(const KF_I32Mat_t* m_, KF_I32Mat_t* result_);

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
static KF_I32ColVec_t KF_MeasurementVector 		= {0};

static KF_I32ColVec_t KF_KalmanGain 		 = {0};
static KF_I32Mat_t 	  KF_CorrectedErrorInEst = {0};
static KF_I32Mat_t	  KF_PredictedErrorInEst = {0};
static KF_I32ColVec_t KF_CorrectedStateEst 	 = {0};
static KF_I32ColVec_t KF_PredictedStateEst 	 = {0};

static int32_t KF_UnscaledEstimatedVel = 0;
static int32_t KF_UnscaledEstimatedPos = 0;

static int32_t KF_Residuum 			  = 0; //can be local?
static int32_t KF_PositionMeasurement = 0;
static int32_t KF_Denominator 		  = 0; //can be local?
static int16_t KF_ModuloCntr 		  = 0;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void KF_UpdateMeasurement()
{
	KF_PositionMeasurement = Q4CLeft_GetPos();
}

static void KF_UpdateUnscaledVals()
{
	KF_UnscaledEstimatedPos = (KF_ModuloCntr*((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) + KF_CorrectedStateEst.aRow[0])/((int32_t)KF_SCALE_X);
	KF_UnscaledEstimatedVel = KF_CorrectedStateEst.aRow[1]/(int32_t)KF_SCALE_X;
}

static void KF_UpdateModuloCounter()
{
	if(KF_ModuloCntr > 0)
	{
		if((KF_CorrectedStateEst.aRow[0] >= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((((int32_t)KF_SCALE_X)*KF_PositionMeasurement) >=  KF_ModuloCntr*(int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)))
		{
			KF_CorrectedStateEst.aRow[0] %= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A);
			KF_ModuloCntr++;
		}
		else if((KF_CorrectedStateEst.aRow[0] <= 0) && (((int32_t)KF_SCALE_X)*KF_PositionMeasurement) <= KF_ModuloCntr*(int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))
		{
			KF_CorrectedStateEst.aRow[0] = ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))-1;
			KF_ModuloCntr--;
		}
	}
	else if(KF_ModuloCntr == 0)
	{
		if((KF_CorrectedStateEst.aRow[0] >= (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)) && ((((int32_t)KF_SCALE_X)*KF_PositionMeasurement) >=  (int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)))
		{
			KF_CorrectedStateEst.aRow[0] %= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A));
			KF_ModuloCntr++;
		}
		else if((KF_CorrectedStateEst.aRow[0] <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))) && (((int32_t)KF_SCALE_X*KF_PositionMeasurement) <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
		{
			KF_CorrectedStateEst.aRow[0] *= (-1);
			KF_CorrectedStateEst.aRow[0] %= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A));
			KF_CorrectedStateEst.aRow[0] *= (-1);
			KF_ModuloCntr--;
		}
	}
	else if(KF_ModuloCntr < 0)
	{
		if((KF_CorrectedStateEst.aRow[0] <= -((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))) && ((((int32_t)KF_SCALE_X)*KF_PositionMeasurement) <=  KF_ModuloCntr*((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))))
		{
			KF_CorrectedStateEst.aRow[0] *= (-1);
			KF_CorrectedStateEst.aRow[0] %= ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A));
			KF_CorrectedStateEst.aRow[0] *= (-1);
			KF_ModuloCntr--;
		}
		else if((KF_CorrectedStateEst.aRow[0] >= 0) && (((int32_t)KF_SCALE_X)*KF_PositionMeasurement) >= KF_ModuloCntr*((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)))
		{
			KF_CorrectedStateEst.aRow[0] = (-((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A)))+1;
			KF_ModuloCntr++;
		}
	}
}

static void KF_SetInitialValues()

{
	uint8_t i = 0u;
	uint8_t j = 0u;
	for(i = 0u; i < KF_SYS_DIMENSION; i++)
	{
		KF_CorrectedStateEst.aRow[i] = kfCfg->StateInitialEstimate->aRow[i];
	}
	for(i = 0u; i < KF_SYS_DIMENSION; i++)
	{
		for(j = 0u; j < KF_SYS_DIMENSION; j++)
		{
			KF_CorrectedErrorInEst.aRow[i].aCol[j] = kfCfg->InitialErrorInEstimate->aRow[i].aCol[j];
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
}

void KF_Main()
{
	StdRtn_t retVal = ERR_OK;

	KF_I32ColVec_t P_times_c	    = {0};
	KF_I32ColVec_t K_times_Residuum = {0};
	KF_I32ColVec_t tempColVec 		= {0};
	KF_I32Mat_t Ident_minus_K_times_cT = {0};
	KF_I32Mat_t tempMat  = {0};
	KF_I32Mat_t tempMat2 = {0};
	KF_I32Mat_t tempMat3 = {0};
	int32_t tempResult = 0;

	KF_UpdateModuloCounter();

	/* Time Update / "predictor" */
		//x_k_hat
		retVal |= KF_MultMatColVec(kfCfg->SystemMatrix, &KF_CorrectedStateEst, &KF_PredictedStateEst, (int32_t)KF_SCALE_A); //scale x

		//P_k
		retVal |= KF_MultMatrices(kfCfg->SystemMatrix, &KF_CorrectedErrorInEst, &tempMat, (int32_t)KF_SCALE_A);
		retVal |= KF_MultMatrices(&tempMat, &KF_SystemMatrixTransposed, &tempMat2, (int32_t)KF_SCALE_A);
		retVal |= KF_AddMatrices(&tempMat2, kfCfg->ProcessNoiseCov, &KF_PredictedErrorInEst, FALSE); //ERROR IN ESTIMATE

	/* Measurement Update / "corrector" */
		//KalmanGain
		retVal |= KF_MultMatColVec(&KF_PredictedErrorInEst, &KF_MeasurementVector, &P_times_c, 1); 		//numerator
		retVal |= KF_MultRowVecColVec(kfCfg->MeasurementVectorTransposed, &P_times_c, &tempResult);		//denominator
		KF_Denominator = (tempResult + (*kfCfg->MeasurementNoiseCov));  								//denominator

		P_times_c.aRow[0] *= (int32_t)KF_SCALE_KALMANGAIN; //scale numerator
		P_times_c.aRow[1] *= (int32_t)KF_SCALE_KALMANGAIN;

		retVal |= KF_MultColVecFactor(&P_times_c, KF_Denominator, &KF_KalmanGain, TRUE);

		//x_k_hat
		KF_Residuum = ((((int32_t)KF_SCALE_X)*KF_PositionMeasurement) % ((int32_t)(KF_MAX_POS_VAL/KF_SCALE_A))) - KF_CorrectedStateEst.aRow[0];
		retVal |= KF_MultColVecFactor(&KF_KalmanGain, KF_Residuum, &K_times_Residuum, FALSE);
		retVal |= KF_MultColVecFactor(&K_times_Residuum, (int32_t)KF_SCALE_KALMANGAIN, &tempColVec, TRUE);
		retVal |= KF_AddColVecs(&KF_PredictedStateEst, &tempColVec, &KF_CorrectedStateEst, FALSE);

		KF_UpdateMeasurement();

		//P_k
		retVal |= KF_MultColVecRowVec(&KF_KalmanGain, kfCfg->MeasurementVectorTransposed, &tempMat3);
		retVal |= KF_AddMatrices(kfCfg->IdentityMatrix, &tempMat3, &Ident_minus_K_times_cT, TRUE);
		retVal |= KF_MultMatrices(&Ident_minus_K_times_cT, &KF_PredictedErrorInEst, &KF_CorrectedErrorInEst, (int32_t)KF_SCALE_KALMANGAIN);

	if(ERR_OK != retVal)
	{
		for(;;){} //error case
	}

	KF_UpdateUnscaledVals();

}

#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
