/***********************************************************************************************//**
 * @file		kf_cfg.c
 * @ingroup		kf
 * @brief 		This file contains all matrix and scaling definitions for the Kalman-Filter
 *
 * In this config file you can define the matrices for your application. A Kalman Filter algorithm is used
 * that calculates the optimal state estimate x_hat(k+1|k) for the timestep 'k+1' given 'k' previous values
 * for an n-dimensional system of the form:
 *
 * x(k+1|k) = A*x(k|k-1) + B*u(k) + w(k) ,
 * y(k)		= C*x(k|k-1) + v(k) .
 *
 * In this, 'A' is the n-by-n system matrix, 'B' is the n-by-l input matrix (l being the number of inputs),
 * 'C' is the m-by-n measurement matrix ('m' being the number of measured states). Additionally,
 * 'u(k)' describes the input to the system, 'w(k)' describes the disturbances to the system
 * and should be normally distributed with zero mean and standard deviation 'sigma_w'. The measured
 * states are described by 'y(k)'. Disturbances of the measurement are expressed with 'v(k)', which
 * should be normally distributed with zero mean and standard deviation 'sigma_v'.
 *
 * The optimal state estimate x_hat_(k+1|k) is then calculated according to the following algorithm:
 *
 * x_hat(k+1|k) = A*x_hat(k|k)  + B*u(k) ,
 * P(k+1|k)	    = A*P_(k|k)*A^T + Q ,
 *
 * with
 *
 * K(k)       = P(k|k-1)*C^T * (C*P(k|k-1)*C^T + R)^{-1} ,
 * x_hat(k|k) = x_hat(k|k-1) + K(k)*( y(k) - C*x_hat(k|k-1) ) ,
 * P(k|k)     = (E - K(k)*C) * P(k|k-1).
 *
 * 'Q' is a n-by-n diagonal matrix containing the variances 'sigma_w^2' of the process disturbances for each state.
 * 'R' is a m-by-m diagonal matrix containing the variances 'sigma_v^2' of the measurement disturbances for each
 * state. K(k) is the n-by-m Kalman gain matrix and P the n-by-n error covariance matrix. The initial conditions
 * are x(0|-1) = x0 = 0 and P(0|-1) = P0 = alpha*eye(n) with alpha >> 1.
 *
 * *
 * @author G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author S. Helling,  stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	26.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_KF_CFG_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "kf_cfg.h"

/** Application APIs **/
#include "tacho_api.h"

/*======================================= >> #DEFINES << =========================================*/
/**
 *  Default scaling values. Number defines left shift
 */
#define KF_DFLT_SCL_A (10)  //1024
#define KF_DFLT_SCL_X (7)   //128
#define KF_DFLT_SCL_ERR (6) //64


/**
 * Defines the default maximum modulo value for measured values
 */
#define KF_DFLT_MAX_MOD_VAL (2000000000u)


/**
 *	macros to initialize matrices. careful with 'A' and 'AT' (division with '1000')
 */
#define MTX_INIT_IDENT {{1, 0}, {0, 1}}
#define MTX_INIT_A(a11_, a12_, a21_, a22_) {{a11_<<KF_DFLT_SCL_A, (a12_<<KF_DFLT_SCL_A)/1000}, {a21_<<KF_DFLT_SCL_A, a22_<<KF_DFLT_SCL_A}}
#define MTX_INIT_AT(at11_, at12_, at21_, at22_) {{at11_<<KF_DFLT_SCL_A, (at12_<<KF_DFLT_SCL_A)}, {(at21_<<KF_DFLT_SCL_A)/1000, at22_<<KF_DFLT_SCL_A}}
#define MTX_INIT_B(b11_, b12_, b21_, b22_) {{b11_<<KF_DFLT_SCL_X, b12_<<KF_DFLT_SCL_X}, {b21_<<KF_DFLT_SCL_X, b22_<<KF_DFLT_SCL_X}}
#define MTX_INIT_R(r11_, r12_, r21_, r22_) {{r11_<<KF_DFLT_SCL_ERR, r12_<<KF_DFLT_SCL_ERR}, {r21_<<KF_DFLT_SCL_ERR, r22_<<KF_DFLT_SCL_ERR}}
#define MTX_INIT_Q(q11_, q12_, q21_, q22_) {{q11_<<KF_DFLT_SCL_ERR, q12_<<KF_DFLT_SCL_ERR}, {q21_<<KF_DFLT_SCL_ERR, q22_<<KF_DFLT_SCL_ERR}}


/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/
/**
 * matrix config definitions for initialization process
 */
static int32_t A[2][2]  = MTX_INIT_A(1, TACHO_SAMPLE_PERIOD_MS, 0, 1);
static int32_t AT[2][2] = MTX_INIT_AT(1, 0, TACHO_SAMPLE_PERIOD_MS, 1);
static int32_t B[2][2]  = MTX_INIT_B(0, 0, 0, 0);
static int32_t C[2][2]  = MTX_INIT_IDENT;
static int32_t CT[2][2] = MTX_INIT_IDENT;
static int32_t R[2][2]  = MTX_INIT_R(3, 0, 0, 20000);
static int32_t Q[2][2]  = MTX_INIT_Q(10, 0, 0, 2500);

/**
 *  runtime data matrices
 */
static int32_t vPrvStEstLe[2][1]    = {0};
static int32_t vOptStEstLe[2][1]    = {0};
static int32_t mPrvErrCoVarLe[2][2] = {0};


static int32_t vPrvStEstRi[2][1]    = {0};
static int32_t vOptStEstRi[2][1]    = {0};
static int32_t mPrvErrCoVarRi[2][2] = {0};



/**
 *	tacho left
 */
static KF_MtxCfg_t mtxCfgLe = {{A[0], 2, 2}, {AT[0], 2, 2}, {B[0], 2, 2}, {C[0], 2, 2}, {CT[0], 2, 2},
							   {R[0], 2, 2}, { Q[0], 2, 2}};
static KF_SclCfg_t sclCfgLe = {KF_DFLT_SCL_A, KF_DFLT_SCL_ERR, KF_DFLT_SCL_X, KF_DFLT_MAX_MOD_VAL};
static KF_DimCfg_t dimCfgLe = {2, 0, 2};
static KF_ReadFct_t KF_MeasValFctHdlsLe[2] = {TACHO_Read_PosLe, KF_Read_Rawi32SpdLe};
static KF_Data_t dataLe = { {vPrvStEstLe[0], 2, 1}, {vOptStEstLe[0], 2, 1}, {mPrvErrCoVarLe[0], 2, 2}, 0 };

/**
 * tacho right
 */
static KF_MtxCfg_t mtxCfgRi = {{A[0], 2, 2}, {AT[0], 2, 2}, {B[0], 2, 2}, {C[0], 2, 2}, {CT[0], 2, 2},
							   {R[0], 2, 2}, { Q[0], 2, 2}};
static KF_SclCfg_t sclCfgRi = {KF_DFLT_SCL_A, KF_DFLT_SCL_ERR, KF_DFLT_SCL_X, KF_DFLT_MAX_MOD_VAL};
static KF_DimCfg_t dimCfgRi = {2, 0, 2};
static KF_ReadFct_t KF_MeasValFctHdlsRi[2] = {TACHO_Read_PosRi, KF_Read_Rawi32SpdRi};
static KF_Data_t dataRi = { {vPrvStEstRi[0], 2, 1}, {vOptStEstRi[0], 2, 1}, {mPrvErrCoVarRi[0], 2, 2}, 0 };


/**
 *
 */
static KF_Itm_t KF_Items[] =
{
		{
				{TACHO_OBJECT_STRING(TACHO_ID_LEFT), TACHO_SAMPLE_PERIOD_MS, &mtxCfgLe,
				&sclCfgLe, &dimCfgLe, KF_MeasValFctHdlsLe, NULL}, &dataLe
		},
		{
				{TACHO_OBJECT_STRING(TACHO_ID_RIGHT), TACHO_SAMPLE_PERIOD_MS, &mtxCfgRi,
				&sclCfgRi, &dimCfgRi, KF_MeasValFctHdlsRi, NULL}, &dataRi
		},
};

/**
 *
 */
static KF_ItmTbl_t KF_ItemTable =
{
	KF_Items,
	sizeof(KF_Items)/(sizeof(KF_Items[0])),
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
KF_ItmTbl_t *Get_pKfItmTbl(void) {return &KF_ItemTable;}


/**
 * wrapper function definitions
 */
StdRtn_t KF_Read_Rawi32SpdLe(int32_t *spd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int16_t tempSpd = 0;
	if(NULL != spd_)
	{
		retVal  = ERR_OK;
		retVal |= TACHO_Read_RawSpdLe(&tempSpd);
		*spd_   = (int32_t)tempSpd;
	}
	else
	{
		/* error handling */
	}
	return retVal;
}

StdRtn_t KF_Read_Rawi32SpdRi(int32_t *spd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int16_t tempSpd = 0;
	if(NULL != spd_)
	{
		retVal  = ERR_OK;
		retVal |= TACHO_Read_RawSpdRi(&tempSpd);
		*spd_   = (int32_t)tempSpd;
	}
	else
	{
		/* error handling */
	}
	return retVal;
}

#ifdef MASTER_KF_CFG_C_
#undef MASTER_KF_CFG_C_
#endif /* !MASTER_KF_CFG_C_ */
