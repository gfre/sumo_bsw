/***********************************************************************************************//**
 * @file		kf_cfg.c
 * @ingroup		kf
 * @brief 		This file contains all matrix and scaling definitions for the Kalman-Filter
 *
 * In this config file you can define the matrices for your application. A Kalman Filter algorithm is used
 * that calculates the optimal state estimate x_hat(k+1|k) for the timestep 'k+1' given 'k' previous values
 * for an n-dimensional system of the form:
 *
 * x(k+1|k) = Phi*x(k|k-1) + Gamma*u(k) + w(k) ,
 * y(k)		= H*x(k|k-1) + v(k) .
 *
 * The optimal state estimate x_hat(k+1|k) is then calculated according to the following  predictor-corrector algorithm,
 * where x_apri denotes the apriori state estimate prior to taking measurements into account, and x_apost denotes the
 * state estimate after taking measurements into account. x_apost(-) is the a posteriori estimate from the previous step,
 * x_apost(+) is the a posteriori estimate from the current step. Same goes for the error covariance matrix P.
 *
 * Predictor
 * x_apri = Phi*x_apost(-)  + Gamma*u(k) ,
 * P_apri = Phi*P_apost(-)*Phi' + G*Q*G' ,
 *
 * Corrector
 * K(k)       = P_apri*H' * (H*P_apri*H' + R)^{-1} ,
 * x_apost(+) = x_apri + K(k)*( y(k) - H*x_apri ) ,
 * P_apost(+) = (E - K(k)*H) * P_apri.
 *
 *
 * Dimensions
 *  x     n-by-1 state vector,
 *  Phi   n-by-n system/state transition matrix,
 *  P(k)  n-by-n error covariance matrix,
 *  u(k)  l-by-1 input vector to the system system,
 *  Gamma n-by-l input matrix (l being the number of inputs),
 *  y(k)  m-by-1 measurement vector,
 *  H     m-by-n measurement matrix ('m' being the number of measured states),
 *  w(k)  n-by-1 disturbance vector to the system (normally distributed with zero mean and standard deviation 'sigma_w'),
 *  G     n-by-n coupling matrix for process noise,
 *  Q     n-by-n diagonal matrix containing the variances 'sigma_w^2' of the process disturbances for each state,
 *  v(k)  m-by-1 measurement noise vector (normally distributed with zero mean and standard deviation 'sigma_v',
 *  R     m-by-m diagonal matrix containing the variances 'sigma_v^2' of the measurement disturbances for each state,
 *  K(k)  n-by-m Kalman gain matrix,
 *  E     n-by-n identity matrix.
 *
 * The prediction for the error covariance matrix P_apri is calculated according to C. Thornton, using UD-factors of P_apost(-).
 * The correction for the error covariance matrix P_apost(+) is calculated according to G. Bierman, using UD-factors of P_apri.
 *
 *
 * @author G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	26.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_KF_CFG_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "kf_cfg.h"
#include "fix16.h"

/** Application APIs **/
#include "tacho_api.h"

/*======================================= >> #DEFINES << =========================================*/
/**
 *  Dimensions for Kalman filters
 */
#define KF_TACHO_SYS_LE (2u)
#define KF_TACHO_MSRMNTS_LE (2u)
#define KF_TACHO_INPTS_LE (0u)

#define KF_TACHO_SYS_RI (2u)
#define KF_TACHO_MSRMNTS_RI (2u)
#define KF_TACHO_INPTS_RI (0u)

/**
 * Defines the default maximum modulo value for measured values
 */
#define KF_DFLT_MAX_MOD_VAL (2000000000u)


/**
 *	macros to initialize matrices
 */
#define MTX_INIT_2X2(rows_, columns_, m11_, m12_, m21_, m22_) {rows_, columns_, 0, {{m11_, m12_}, {m21_, m22_}}}


/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/
/**
 *	function handles for left tacho measurements
 */
static KF_ReadFct_t KF_MeasValFctHdlsLe[KF_TACHO_MSRMNTS_LE] = {TACHO_Read_PosLe, KF_Read_Rawi32SpdLe};

/**
 *  function handles for right tacho measurements
 */
static KF_ReadFct_t KF_MeasValFctHdlsRi[KF_TACHO_MSRMNTS_RI] = {TACHO_Read_PosRi, KF_Read_Rawi32SpdRi};


/**
 *
 */
static KF_Itm_t KF_Items[] =
{
/*======================== tacho left =========================*/
			{
/* Config */	{
	/* Name */			TACHO_OBJECT_STRING(TACHO_ID_LEFT),
	/* Sample Time */	TACHO_SAMPLE_PERIOD_MS,
		/* Matrices */{
		/* Phi */		MTX_INIT_2X2(KF_TACHO_SYS_LE,     KF_TACHO_SYS_LE,     1<<16, ((TACHO_SAMPLE_PERIOD_MS<<16)/1000), 0, 1<<16),
		/* Gamma */		{0u},
			/* C */		MTX_INIT_2X2(KF_TACHO_MSRMNTS_LE, KF_TACHO_SYS_LE,     1<<16,  0,                                  0, 1<<16),
			/* R */		MTX_INIT_2X2(KF_TACHO_MSRMNTS_LE, KF_TACHO_MSRMNTS_LE, 3<<16,  0, 								   0, 20000<<16),
			/* G */		MTX_INIT_2X2(KF_TACHO_SYS_LE,     KF_TACHO_SYS_LE,     1<<16, 0, 								   0, 1<<16),
			/* Q */		MTX_INIT_2X2(KF_TACHO_SYS_LE,     KF_TACHO_SYS_LE,     10<<16, 0, 								   0, 2500<<16)
					  },
		/* MeasFcts */KF_MeasValFctHdlsLe,
		/* InptFcts */NULL,
  /* Modulo counter */TRUE
				},
/* Data */		{
	/* vXapri  */	{KF_TACHO_SYS_LE, 1, 0, {0u}},
	/* vXapost */ 	{KF_TACHO_SYS_LE, 1, 0, {0u}},
	/* mUPapri */	{KF_TACHO_SYS_LE, KF_TACHO_SYS_LE, 0, {0u}},
	/* mDPapri */	{KF_TACHO_SYS_LE, KF_TACHO_SYS_LE, 0, {0u}},
	/* mUPapost */	{KF_TACHO_SYS_LE, KF_TACHO_SYS_LE, 0, {0u}},
	/* mDPapost */	{KF_TACHO_SYS_LE, KF_TACHO_SYS_LE, 0, {0u}},
	/* nMdCntr */	0
				}
			},
/*======================== tacho right =========================*/
			{
/* Config */	{
	/* Name */			TACHO_OBJECT_STRING(TACHO_ID_RIGHT),
	/* Sample Time */	TACHO_SAMPLE_PERIOD_MS,
		/* Matrices */{
		/* Phi */		MTX_INIT_2X2(KF_TACHO_SYS_RI,     KF_TACHO_SYS_RI,     1<<16, ((TACHO_SAMPLE_PERIOD_MS<<16)/1000), 0, 1<<16),
		/* Gamma */		{0u},
			/* H */		MTX_INIT_2X2(KF_TACHO_MSRMNTS_RI, KF_TACHO_SYS_RI,     1<<16,  0,                                  0, 1<<16),
			/* R */		MTX_INIT_2X2(KF_TACHO_MSRMNTS_RI, KF_TACHO_MSRMNTS_RI, 3<<16,  0, 								   0, 20000<<16),
			/* G */		MTX_INIT_2X2(KF_TACHO_SYS_RI,     KF_TACHO_SYS_RI,     1<<16, 0, 								   0, 1<<16),
			/* Q */		MTX_INIT_2X2(KF_TACHO_SYS_RI,     KF_TACHO_SYS_RI,     10<<16, 0, 								   0, 2500<<16)
					  },
		/* MeasFcts */KF_MeasValFctHdlsRi,
		/* InptFcts */NULL,
  /* Modulo counter */TRUE
				},
/* Data */		{
	/* vXapri  */	{KF_TACHO_SYS_RI, 1, 0, {0u}},
	/* vXapost */ 	{KF_TACHO_SYS_RI, 1, 0, {0u}},
	/* mUPapri */	{KF_TACHO_SYS_RI, KF_TACHO_SYS_RI, 0, {0u}},
	/* mDPapri */	{KF_TACHO_SYS_RI, KF_TACHO_SYS_RI, 0, {0u}},
	/* mUPapost */	{KF_TACHO_SYS_RI, KF_TACHO_SYS_RI, 0, {0u}},
	/* mDPapost */	{KF_TACHO_SYS_RI, KF_TACHO_SYS_RI, 0, {0u}},
	/* nMdCntr */	0
				}
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
