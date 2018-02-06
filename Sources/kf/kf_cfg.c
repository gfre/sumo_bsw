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
 *	function handles for left tacho
 */
static KF_ReadFct_t KF_MeasValFctHdlsLe[KF_TACHO_MSRMNTS_LE] = {TACHO_Read_PosLe, KF_Read_Rawi32SpdLe};

/**
 *  function handles for right tacho
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
			/* A */		MTX_INIT_2X2(KF_TACHO_SYS_LE,     KF_TACHO_SYS_LE,     1<<16, ((TACHO_SAMPLE_PERIOD_MS<<16)/1000), 0, 1<<16),
			/* B */		MTX_INIT_2X2(0,                   0,                   0,      0,                                  0, 0),
			/* C */		MTX_INIT_2X2(KF_TACHO_MSRMNTS_LE, KF_TACHO_SYS_LE,     1<<16,  0,                                  0, 1<<16),
			/* R */		MTX_INIT_2X2(KF_TACHO_MSRMNTS_LE, KF_TACHO_MSRMNTS_LE, 3<<16,  0, 								   0, 20000<<16),
			/* Q */		MTX_INIT_2X2(KF_TACHO_SYS_LE,     KF_TACHO_SYS_LE,     10<<16, 0, 								   0, 2500<<16)
					  },
		/* Dim */	  {KF_TACHO_SYS_LE, 0, KF_TACHO_MSRMNTS_LE},
		/* MeasFcts */KF_MeasValFctHdlsLe,
		/* InptFcts */NULL
				},
/* Data */		{
	/* vXapri  */	{KF_TACHO_SYS_LE, 1, 0, {0u}},
	/* vXapost */ 	{KF_TACHO_SYS_LE, 1, 0, {0u}},
	/* mUPapri */	{KF_TACHO_SYS_LE, KF_TACHO_SYS_LE, 0, {0u}},
	/* mDPapri */	{KF_TACHO_SYS_LE, KF_TACHO_SYS_LE, 0, {0u}},
	/* mUPapost */	{KF_TACHO_SYS_LE, KF_TACHO_SYS_LE, 0, {{1<<16, 13107},{0 , 1<<16}}},
	/* mDPapost */	{KF_TACHO_SYS_LE, KF_TACHO_SYS_LE, 0, {{96<<16, 0},{0, 100<<16}}},
	/* nMdCntr */	0
				}
			},
/*======================== tacho right =========================*/
			{
/* Config */	{
	/* Name */			TACHO_OBJECT_STRING(TACHO_ID_RIGHT),
	/* Sample Time */	TACHO_SAMPLE_PERIOD_MS,
		/* Matrices */{
			/* A */		MTX_INIT_2X2(KF_TACHO_SYS_RI,     KF_TACHO_SYS_RI,     1<<16, ((TACHO_SAMPLE_PERIOD_MS<<16)/1000), 0, 1<<16),
			/* B */		MTX_INIT_2X2(0,                   0,                   0,      0,                                  0, 0),
			/* C */		MTX_INIT_2X2(KF_TACHO_MSRMNTS_RI, KF_TACHO_SYS_RI,     1<<16,  0,                                  0, 1<<16),
			/* R */		MTX_INIT_2X2(KF_TACHO_MSRMNTS_RI, KF_TACHO_MSRMNTS_RI, 3<<16,  0, 								   0, 20000<<16),
			/* Q */		MTX_INIT_2X2(KF_TACHO_SYS_RI,     KF_TACHO_SYS_RI,     10<<16, 0, 								   0, 2500<<16)
					  },
		/* Dim */	  {KF_TACHO_SYS_RI, 0, KF_TACHO_MSRMNTS_RI},
		/* MeasFcts */KF_MeasValFctHdlsRi,
		/* InptFcts */NULL
				},
/* Data */		{
	/* vXapri  */	{KF_TACHO_SYS_RI, 1, 0, {0u}},
	/* vXapost */ 	{KF_TACHO_SYS_RI, 1, 0, {0u}},
	/* mUPapri */	{KF_TACHO_SYS_RI, KF_TACHO_SYS_LE, 0, {0u}},
	/* mDPapri */	{KF_TACHO_SYS_RI, KF_TACHO_SYS_LE, 0, {0u}},
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
