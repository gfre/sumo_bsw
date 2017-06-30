/*************************************************************************************************
 * @file		kf_cgf.c
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	<I>. <Surname>, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	23.06.2017
 *
 * @copyright @<LGPLx_x>
 *
 ***************************************************************************************************/

#define MASTER_KF_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "kf.h"
#include "tacho_api.h"
#include "kf_cfg.h"
#include "Q4CLeft.h"

/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
//Matrix operations
static StdRtn_t KF_AddMatrices(const KF_I32Mat_t* m_, const KF_I32Mat_t* n_, KF_I32Mat_t* result_, bool subtract_);
static StdRtn_t KF_MultMatrices(const KF_I32Mat_t* m_, const KF_I32Mat_t* v_, KF_I32Mat_t* result_, uint16_t divider_);
static StdRtn_t KF_TransposeMat(const KF_I32Mat_t* m_, KF_I32Mat_t* result_);

//Matrix-Vector operations
static StdRtn_t KF_MultRowVecMat(const KF_I32RowVec_t* v_, const KF_I32Mat_t* M_, KF_I32RowVec_t* result_);
static StdRtn_t KF_MultMatColVec(const KF_I32Mat_t* m_, const KF_I32ColVec_t* n_, KF_I32ColVec_t* result_, uint16_t divider_);

//Vector operations
static StdRtn_t KF_MultRowVecColVec(const KF_I32RowVec_t* v_, const KF_I32ColVec_t* w_, int32_t* result_);
static StdRtn_t KF_TransposeRowVec(const KF_I32RowVec_t* v_, KF_I32ColVec_t* result_);
static StdRtn_t KF_MultColVecFactor(const KF_I32ColVec_t* v_, const int32_t factor_, KF_I32ColVec_t* result_, bool divide_);
static StdRtn_t KF_AddColVecs(const KF_I32ColVec_t* v_,const KF_I32ColVec_t* w_, KF_I32ColVec_t* result_, bool subtract_);
static StdRtn_t KF_MultColVecRowVec(const KF_I32ColVec_t* v_, const KF_I32RowVec_t* w_, KF_I32Mat_t* result_);

static void KF_SetInitialValues(void);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static const KF_Cfg_t* kfCfg = NULL;

static KF_I32Mat_t KF_ATransposed = {0}; 	//Transposed system matrix
static KF_I32ColVec_t KF_KalmanGain = {0}; 	//Kalman Gain vector
static KF_I32Mat_t KF_P_k = {0};; 			//Error in estimate covariance matrix
static KF_I32Mat_t KF_P_k_m = {0}; 			//A priori error in estimate covariance matrix
static KF_I32ColVec_t KF_x_k_hat = {0}; 	//State estimate vector
static KF_I32ColVec_t KF_x_k_hat_m = {0};	// A priori state estimate vector
static KF_I32ColVec_t KF_c = {0};			// System easurement vector

static int32_t KF_currSpeed = 0, KF_currDelta = 0 , KF_ResidualTerm = 0, KF_z_k = 0;

static bool doFirstIteration = TRUE;

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

static StdRtn_t KF_MultMatColVec(const KF_I32Mat_t* M_, const KF_I32ColVec_t* v_, KF_I32ColVec_t* result_, uint16_t divider_) //2x2
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
				result_->aRow[i] += (M_->aRow[i].aCol[j] * v_->aRow[j])/((uint32_t)divider_);
			}
		}

	}

	return retVal;
}


static StdRtn_t KF_MultMatrices(const KF_I32Mat_t* M_, const KF_I32Mat_t* N_, KF_I32Mat_t* result_, uint16_t divider_)
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
					result_->aRow[i].aCol[j] += (M_->aRow[i].aCol[k] * N_->aRow[k].aCol[j])/((uint32_t)divider_);
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

static StdRtn_t KF_AddMatrices(const KF_I32Mat_t* M_, const KF_I32Mat_t* N_, KF_I32Mat_t* result_, bool subtract_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	uint8_t j = 0u;
	if(NULL != result_)
	{
		retVal = ERR_OK;
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
					result_->aRow[i].aCol[j] = M_->aRow[i].aCol[j] + M_->aRow[i].aCol[j];
				}
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
				result_->aRow[i] = (factor_*v_->aRow[i]);
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

static void KF_UpdateMeasurement()
{
	KF_z_k = Q4CLeft_GetPos();//TACHO_GetPositionDelta(TRUE);
}

static void KF_SetInitialValues()
{
	uint8_t i = 0u;
	uint8_t j = 0u;
	for(i = 0u; i < KF_SYS_DIMENSION; i++)
	{
		KF_x_k_hat.aRow[i] = kfCfg->InitialEstimate->aRow[i];
	}
	for(i = 0u; i < KF_SYS_DIMENSION; i++)
	{
		for(j = 0u; j < KF_SYS_DIMENSION; j++)
		{
			KF_P_k.aRow[i].aCol[j] = kfCfg->InitialErrorCovarianceMatrix->aRow[i].aCol[j];
		}
	}
	doFirstIteration = FALSE;
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
	retVal |= KF_TransposeMat(kfCfg->SystemMatrix, &KF_ATransposed);
	retVal |= KF_TransposeRowVec(kfCfg->MeasurementVector, &KF_c);
	if(ERR_OK != retVal)
	{
		for(;;){}
	}
}

void KF_Main()
{
	StdRtn_t retVal = ERR_OK;
	if(TRUE == doFirstIteration)
	{
		KF_SetInitialValues();
	}

	KF_I32Mat_t tempMat= {0};
	KF_I32Mat_t tempMat2 = {0};
	KF_I32Mat_t tempMat3 = {0};
	KF_I32Mat_t tempMat4 = {0};
	KF_I32ColVec_t tempColVec = {0};
	KF_I32ColVec_t tempColVec2 = {0};
	KF_I32ColVec_t tempColVec3 = {0};
	KF_I32RowVec_t tempRowVec = {0};
	int32_t tempResult = 0;

	KF_ResidualTerm = 0;
	/* Time Update, "predictor" */
		//x_k_hat
		uint16_t divider = 1000u;
		retVal |= KF_MultMatColVec(kfCfg->SystemMatrix, &KF_x_k_hat, &KF_x_k_hat_m, divider);	//x_hat_k_m = A*x_k-1_hat_m  -> 2x1

		//P_k
		retVal |= KF_MultMatrices(kfCfg->SystemMatrix, &KF_P_k, &tempMat, divider);	/* P_k_m = A*P_Initial_Error ... tempMat -> 2x2*/
		retVal |= KF_MultMatrices(&tempMat, &KF_ATransposed, &KF_P_k_m, divider);										/* ... *ATransposed  P_k_m -> 2x2*/

	/* Measurement Update / "corrector" */
		//KalmanGain
		retVal |= KF_MultMatColVec(&KF_P_k_m, &KF_c, &tempColVec, 1u); 					// tempColVec = P_k_m * c -> 2x1
		retVal |= KF_MultRowVecMat(kfCfg->MeasurementVector, &KF_P_k_m, &tempRowVec);	// c^T * P_k_m ...   -> 1x2
		retVal |= KF_MultRowVecColVec(&tempRowVec, &KF_c, &tempResult);					//  ... * c  = tempResult -> 1x1
		retVal |= KF_MultColVecFactor(&tempColVec, 1000u, &tempColVec2,FALSE); 			//scale up!
		retVal |= KF_MultColVecFactor(&tempColVec2, (tempResult + *kfCfg->MeasurementNoiseCovariance), &KF_KalmanGain, TRUE);

	KF_UpdateMeasurement();

		//x_k_hat
		KF_ResidualTerm = KF_z_k - KF_x_k_hat_m.aRow[0];
		retVal |= KF_MultColVecFactor(&KF_KalmanGain, KF_ResidualTerm, &tempColVec2, FALSE);//tempColVec2 = KalmanGain * KF_ResidualTerm
		retVal |= KF_MultColVecFactor(&tempColVec2, divider, &tempColVec3, TRUE); 			//scale down!
		retVal |= KF_AddColVecs(&KF_x_k_hat_m, &tempColVec3, &KF_x_k_hat, FALSE);		 	// x_k_hat = x_k_hat_m + tempColVec2

		//P_k
		retVal |= KF_MultColVecRowVec(&KF_KalmanGain, kfCfg->MeasurementVector, &tempMat3);  // tempMat3 = KalmanGain * c^T
		retVal |= KF_AddMatrices(kfCfg->IdentityMatrix, &tempMat3, &tempMat4, TRUE);  //tempMat4 = eye(KF_SYS_DIMENSION) - tempMat3
		retVal |= KF_MultMatrices(&tempMat4, &KF_P_k_m, &KF_P_k, divider);   				//P_k = tempMat4 * P_k_m

		if(ERR_OK != retVal){
			for(;;){}
		}
}

#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
