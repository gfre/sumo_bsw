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
#include "tacho.h"
#include "kf_cfg.h"

/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static const KF_Cfg_t* kfCfg = NULL;


static KF_Matrix_t KF_ATransposed[] = {
		{0, 0},
		{0, 0},
};

static KF_Matrix_t KF_KalmanGain[] = { // Kalman Gain
		{0,	0},
		{0,	0},
};

static KF_Matrix_t KF_P_k[] = { //error in estimate covariance matrix
		{0,	0},
		{0, 0},
};

static KF_Matrix_t KF_P_k_m[] = { //previous error in estimate covariance matrix
		{0,	0},
		{0, 0},
};

static KF_Vector_t KF_x_k_hat[] =  {
		{0}, //position
		{0}, //velocity
 };


static KF_Vector_t KF_x_k_hat_m[] =  {
		{0}, //position
		{0}, //velocity
 };

static KF_Vector_t KF_z_k[] = //measured data
{
		{0},
		{0},
};

static int32_t KF_currSpeed, KF_currDelta;

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

static void KF_MultMatrices(const KF_Matrix_t* m_, const KF_Matrix_t* n_, KF_Matrix_t* result_) //2x2
{
	if(result_ == NULL)
	{
		return;
	}
	result_->aRow[0] = m_->aRow[0] * n_->aRow[0] + m_->aRow[1] * (++n_)->aRow[0];
	result_->aRow[1] = m_->aRow[0] * (--n_)->aRow[1] + m_->aRow[1] * (++n_)->aRow[1];
	(++result_)->aRow[0] = (++m_)->aRow[0] * (--n_)->aRow[0] + m_->aRow[1] * (++n_)->aRow[0];
	result_->aRow[1] = m_->aRow[0] * (--n_)->aRow[1] + m_->aRow[1] * (++n_)->aRow[1];
}

static void KF_MultMatrixVector(const KF_Matrix_t* m_, const KF_Vector_t* v_, KF_Vector_t* result_, bool divideBy1000)
{
	if(result_ == NULL)
	{
		return;
	}
	if(divideBy1000)
	{
		result_->aCol[0] = m_->aRow[0] * v_->aCol[0] + (m_->aRow[1] * (++v_)->aCol[0])/1000;
		(++result_)->aCol[0] = (++m_)->aRow[0] * (--v_)->aCol[0] + m_->aRow[1] * (++v_)->aCol[0];
	}else
	{
		result_->aCol[0] = m_->aRow[0] * v_->aCol[0] + m_->aRow[1] * (++v_)->aCol[0];
		(++result_)->aCol[0] = (++m_)->aRow[0] * (--v_)->aCol[0] + m_->aRow[1] * (++v_)->aCol[0];
	}
}

static void KF_TransposeMatrix(const KF_Matrix_t* m_, KF_Matrix_t* result_)
{
	if(result_ == NULL)
	{
		return;
	}
	result_->aRow[0] = m_->aRow[0];
	result_->aRow[1] = (++m_)->aRow[0];
	(++result_)->aRow[0] = (--m_)->aRow[1];
	result_->aRow[1] = (++m_)->aRow[1];
}

static void KF_TransposeVector(const KF_Vector_t* v_, KF_Vector_t* result_, bool isLineVector_)
{
	if(result_ == NULL)
	{
		return;
	}
	if(isLineVector_)
	{
		result_->aCol[0] = v_->aCol[0];
		(++result_)->aCol[0] = v_->aCol[1];
	}else
	{
		result_->aCol[0] = v_->aCol[0];
		result_->aCol[1] = (++v_)->aCol[0];
	}
}


static void KF_AddMatrices(const KF_Matrix_t* m_, const KF_Matrix_t* n_, KF_Matrix_t* result_, bool add_)
{
	if(result_ == NULL)
	{
		return;
	}
	if(add_)
	{ //add
		result_->aRow[0] = m_->aRow[0] + n_->aRow[0];
		result_->aRow[1] = m_->aRow[1] + n_->aRow[1];
		(++result_)->aRow[0] = (++m_)->aRow[0] + (++n_)->aRow[0];
		result_->aRow[1] = m_->aRow[1] + n_->aRow[1];
	}else
	{ //subtract
		result_->aRow[0] = m_->aRow[0] - n_->aRow[0];
		result_->aRow[1] = m_->aRow[1] - n_->aRow[1];
		(++result_)->aRow[0] = (++m_)->aRow[0] - (++n_)->aRow[0];
		result_->aRow[1] = m_->aRow[1] - n_->aRow[1];
	}
}
static void KF_AddVectors(const KF_Vector_t* v_,const KF_Vector_t* w_, KF_Vector_t* result_, bool add_)
{
	if(result_ == NULL)
	{
		return;
	}
	result_->aCol[0] = v_->aCol[0] + w_->aCol[0];
	(++result_)->aCol[0] = (++v_)->aCol[0] + (++w_)->aCol[0];
}


static void KF_InvertMatrix(const KF_Matrix_t* m_, KF_Matrix_t* result_)
{
	if(result_ == NULL)
	{
		return;
	}
	int16_t det = m_->aRow[0]*(++m_)->aRow[1] - (--m_)->aRow[1] * (++m_)->aRow[0];
	det = 1/det;

	result_->aRow[0] = det * m_->aRow[1];
	result_->aRow[1] = -(det*(--m_)->aRow[1]);
	(++result_)->aRow[0] = -(det*(++m_)->aRow[0]);
	result_->aRow[1] = det * (--m_)->aRow[0];
}

static void KF_UpdateMeasurements()
{
	KF_currDelta = TACHO_GetPositionDelta(TRUE);
	KF_currSpeed = TACHO_GetSpeed(TRUE);
}

static void KF_SetMeasurementVector()
{
	KF_z_k[0].aCol[0] = KF_currDelta;
	KF_z_k[1].aCol[0] = KF_currSpeed;
}
/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void KF_Init()
{
	kfCfg = GetKFCfg();
	if(kfCfg == NULL)
	{
		for(;;){} //error case
	}

	KF_Matrix_t tempMat[] = INIT_MATRIX_DIM_2;
	KF_Matrix_t tempMat2[] = INIT_MATRIX_DIM_2;
	KF_Matrix_t tempMat3[] = INIT_MATRIX_DIM_2;
	KF_Matrix_t tempMat4[] = INIT_MATRIX_DIM_2;
	KF_Matrix_t tempMat5[] = INIT_MATRIX_DIM_2;
	KF_Vector_t tempVec[] = INIT_VECTOR_DIM_2;
	KF_Vector_t tempVec2[] = INIT_VECTOR_DIM_2;
	KF_Vector_t tempVec3[] = INIT_VECTOR_DIM_2;

	/* Time Update, "predictor" */
		//x_k_hat
		KF_MultMatrixVector(kfCfg->SystemMatrix, kfCfg->InitialEstimate, KF_x_k_hat_m, TRUE);	//x_hat_k_m = A*x_k-1_hat_m
		//P_k
		KF_MultMatrices(kfCfg->SystemMatrix, kfCfg->InitialErrorCovarianceMatrix, tempMat);		/* P_k_m = A*P_Initial_Error ... */
		KF_TransposeMatrix(kfCfg->SystemMatrix, KF_ATransposed);
		KF_MultMatrices(tempMat, KF_ATransposed, KF_P_k_m);										/* ... *ATransposed */


	/* Measurement Update / "corrector" */
		//KalmanGain
		KF_MultMatrices(KF_P_k_m, kfCfg->MeasurementMatrix, tempMat); 						/* tempMat = P_k_m * H (transposed) */
		KF_AddMatrices(KF_P_k_m, kfCfg->MeasurementNoiseCovarianceMatrix, tempMat2, TRUE);	/* tempMat2 = H * P_k_m * H (Transposed) + R */
		KF_InvertMatrix(tempMat2, tempMat3);							/* tempMat3 = (tempMat2)^-1*/
		KF_MultMatrices(tempMat, tempMat3, KF_KalmanGain);				/* KalmanGain = tempMat * tempMat3 = P_k_m * H(transposed) / (H * P_k_m * H(transposed) + R) */

	KF_UpdateMeasurements();
	KF_SetMeasurementVector();

		//x_k_hat
		KF_AddVectors(KF_z_k, KF_x_k_hat_m, tempVec, FALSE);			//tempVec = z_k - H * x_k_hat_m
		KF_MultMatrixVector(KF_KalmanGain, tempVec, tempVec2, FALSE);	//tempvec2 = KalmanGain * tempVec = KalmanGain * (z_k - H * x_k_hat_m)
		KF_AddVectors(KF_x_k_hat_m, tempVec2, tempVec3, TRUE); 			//x_k_hat = x_k_hat_m + KalmanGain(z_k - H * x_k_hat_m);
		KF_x_k_hat[0] = tempVec3[0];
		KF_x_k_hat[1] = tempVec3[1];


		//P_k
		KF_AddMatrices(kfCfg->IdentityMatrix, KF_KalmanGain, tempMat4, FALSE);  //eye(2) - KalmanGain * H
		KF_MultMatrices(tempMat4, KF_P_k_m, tempMat5);
		KF_P_k[0] = tempMat5[0];
		KF_P_k[1] = tempMat5[1];
}

void KF_Main(){
	KF_Matrix_t tempMat[] = INIT_MATRIX_DIM_2;
	KF_Matrix_t tempMat2[] = INIT_MATRIX_DIM_2;
	KF_Matrix_t tempMat3[] = INIT_MATRIX_DIM_2;
	KF_Matrix_t tempMat4[] = INIT_MATRIX_DIM_2;
	KF_Matrix_t tempMat5[] = INIT_MATRIX_DIM_2;
	KF_Vector_t tempVec[] = INIT_VECTOR_DIM_2;
	KF_Vector_t tempVec2[] = INIT_VECTOR_DIM_2;
	KF_Vector_t tempVec3[] = INIT_VECTOR_DIM_2;


	/* Time Update, "predictor" */
		//x_k_hat
		KF_MultMatrixVector(kfCfg->SystemMatrix, KF_x_k_hat, KF_x_k_hat_m, TRUE);	//x_hat_k_m = A*x_k-1_hat_m
		//P_k
		KF_MultMatrices(kfCfg->SystemMatrix, KF_P_k, tempMat);				/* P_k_m = A*P_Initial_Error ... */
		KF_TransposeMatrix(kfCfg->SystemMatrix, KF_ATransposed);
		KF_MultMatrices(tempMat, KF_ATransposed, KF_P_k_m);					/* ... *ATransposed */


	/* Measurement Update / "corrector" */
		//KalmanGain
// =P_k_m		KF_MultMatrices(KF_P_k_m, kfCfg->MeasurementMatrix, tempMat); 				/* tempMat = P_k_m * H (transposed) */
		KF_AddMatrices(KF_P_k_m, kfCfg->MeasurementNoiseCovarianceMatrix, tempMat2, TRUE);	/* tempMat2 = H * P_k_m * H (Transposed) + R */
		KF_InvertMatrix(tempMat2, tempMat3);					/* tempMat3 = (tempMat2)^-1*/
		KF_MultMatrices(KF_P_k_m, tempMat3, KF_KalmanGain);		/* KalmanGain = tempMat * tempMat3 = P_k_m * H(transposed) / (H * P_k_m * H(transposed) + R) */

	KF_UpdateMeasurements();
	KF_SetMeasurementVector();

		//x_k_hat
		KF_AddVectors(KF_z_k, KF_x_k_hat_m, tempVec, FALSE);			//tempVec = z_k - H * x_k_hat_m
		KF_MultMatrixVector(KF_KalmanGain, tempVec, tempVec2, FALSE);	//tempvec2 = KalmanGain * tempVec = KalmanGain * (z_k - H * x_k_hat_m)
		KF_AddVectors(KF_x_k_hat_m, tempVec2, tempVec3, TRUE);			//x_k_hat = x_k_hat_m + KalmanGain(z_k - H * x_k_hat_m);
		KF_x_k_hat[0] = tempVec3[0];
		KF_x_k_hat[1] = tempVec3[1];

		//P_k
		KF_AddMatrices(kfCfg->IdentityMatrix, KF_KalmanGain, tempMat4, FALSE);  //eye(2) - KalmanGain * H
		KF_MultMatrices(tempMat4, KF_P_k_m, tempMat5);
		KF_P_k[0] = tempMat5[0];
		KF_P_k[1] = tempMat5[1];
}


#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
