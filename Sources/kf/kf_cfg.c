/***********************************************************************************************//**
 * @file		kf_cfg.c
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
* In this config file you can set the matrices for your application. A Kalman Filter algorithm is used
 * that calculates a prediction and a correction (estimation) for a system of the form:
 * x(k|k-1) = A*x(k-1|k-2) + B*u(k) + w(k)
 * y(k)		= C*x(k|k-1) + v(k)
 *
 * and estimates the states using the following algorithm:
 *
 * x_hat(k|k-1) = A*x_hat(k-1|k-2) + B*u(k)
 * P(k|k-1)     = A*P(k-1|k-2)*A^T + Q
 *
 * K_k        = P(k|k-1)*C^T * (C*P(k|k-1)C^T + R)^-1
 * x_hat(k|k) = x_hat(k|k-1) + K_k(y(k) - C*x_hat(k|k-1))
 * P(k|k)     = (I - K_k*C) * P(k|k-1)
 *
 * where x(0) = x0 = 0 and P(0) = P0 = alpha*eye(n) with alpha >> 1.
 * *
 * @author 	<I>. <Surname>, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	26.06.2017
 *
 * @copyright @<LGPLx_x>
 *
 ***************************************************************************************************/

#define MASTER_KF_CFG_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "kf_cfg.h"
#include "tacho.h"

/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static KF_I32Mat_t KF_I =
{
		{
				{KF_SCALE_KALMANGAIN, 0},
				{0, KF_SCALE_KALMANGAIN},
		},
};

/*-------------SYSTEM CONFIGURATION-------------*/
	static KF_I32Mat_t KF_A =
	{
			{
					{1*KF_SCALE_A, ((TACHO_SAMPLE_PERIOD_MS*KF_SCALE_A)/1000)},
					{0, 1*KF_SCALE_A},
			},
	};

	static KF_I32Mat_t KF_B =
	{
			{
					{0*KF_SCALE_X, 0*KF_SCALE_X},
					{0*KF_SCALE_X, 0*KF_SCALE_X},
			}
	};


	static KF_I32RowVec_t KF_cT = //measurement Vector
	{
					{1, 0},
	};

	static KF_I32Mat_t KF_C =
	{
			{
					{1, 0},
					{0, 1},
			},
	};

	static KF_I32ColVec_t KF_x0 =  //initial estimate
	{
			{0, 0},
	};

/*-----------COVARIANCE MATRICES------------*/
	static int32_t KF_r = 3*KF_SCALE_ERROR;// measurement noise covariance for s

	static KF_I32Mat_t KF_R =
	{
			{
					{3*KF_SCALE_ERROR, 0},
					{0, 40000*KF_SCALE_ERROR},
			},
	};


	static KF_I32Mat_t KF_Q = // process noise covariance matrix
	{
			{
					{10*KF_SCALE_ERROR, 0},
					{0, 2500*KF_SCALE_ERROR},
			},
	};

	static KF_I32Mat_t KF_P0 = // initial error in estimate covariance matrix
	{
			{
				{100, 0},
				{0, 100},
			},
	};


/*---------CONFIG----------*/
	static KF_Cfg_t kfCfg =
	{
			&KF_I,
			&KF_A,
			&KF_B,
			&KF_cT,
			&KF_C,
			&KF_x0,
			&KF_r,
			&KF_R,
			&KF_Q,
			&KF_P0,
	};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
const KF_Cfg_t* GetKFCfg(void){
	return &kfCfg;
}


#ifdef MASTER_KF_CFG_C_
#undef MASTER_KF_CFG_C_
#endif /* !MASTER_KF_CFG_C_ */
