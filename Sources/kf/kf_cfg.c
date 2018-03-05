/***********************************************************************************************//**
 * @file		kf_cfg.c
 * @ingroup		kf
 * @brief 		This file contains all matrix and scaling definitions for the Kalman filters
 *
 * This module contains the configurations and initializations of all active Kalman filters.
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

#define KF_TACHO_SYS_RI (2u)
#define KF_TACHO_MSRMNTS_RI (2u)


/**
 *	@brief macro to initialize 2-by-2 matrices
 */
#define MTX_INIT_2X2(dim_, m11_, m12_, m21_, m22_) {dim_, dim_, 0, {{m11_, m12_}, {m21_, m22_}}}

/**
 * @brief macro for default runtime data init
 */
#define KF_DFLT_DATA_INIT(n_) { \
								/* vXapri  */	{n_, 1,  0, {0u}},\
								/* vXapost */ 	{n_, 1,  0, {0u}},\
								/* mUPapri */	{n_, n_, 0, {0u}},\
								/* mDPapri */	{n_, n_, 0, {0u}},\
								/* mUPapost */	{n_, n_, 0, {0u}},\
								/* mDPapost */	{n_, n_, 0, {0u}},\
								/* nMdCntr */	{0}\
							  }


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
	/* Matrices */	{
		/* Phi */		MTX_INIT_2X2(KF_TACHO_SYS_LE,     1<<16, ((TACHO_SAMPLE_PERIOD_MS<<16)/1000), 0, 1<<16),
		/* Gamma */		{0u},
		/* H */			MTX_INIT_2X2(KF_TACHO_SYS_LE,     1<<16,  0,                                  0, 1<<16),
		/* R */			MTX_INIT_2X2(KF_TACHO_MSRMNTS_LE, 3<<16,  0, 								  0, 20000<<16),
		/* G */			MTX_INIT_2X2(KF_TACHO_SYS_LE,     1<<16,  0, 								  0, 1<<16),
		/* Q */			MTX_INIT_2X2(KF_TACHO_SYS_LE,     10<<16, 0, 								  0, 2500<<16)
					},
	/* MeasFcts */	KF_MeasValFctHdlsLe,
	/* InptFcts */	NULL,
    /*Modulo counter*/TRUE
				},
/* Data */		KF_DFLT_DATA_INIT(KF_TACHO_SYS_LE)
			},
/*======================== tacho right =========================*/
			{
/* Config */	{
	/* Name */			TACHO_OBJECT_STRING(TACHO_ID_RIGHT),
	/* Sample Time */	TACHO_SAMPLE_PERIOD_MS,
	/* Matrices */	{
		/* Phi */		MTX_INIT_2X2(KF_TACHO_SYS_RI,     1<<16, ((TACHO_SAMPLE_PERIOD_MS<<16)/1000), 0, 1<<16),
		/* Gamma */		{0u},
		/* H */			MTX_INIT_2X2(KF_TACHO_SYS_RI,     1<<16,  0,                                  0, 1<<16),
		/* R */			MTX_INIT_2X2(KF_TACHO_MSRMNTS_RI, 3<<16,  0, 								  0, 20000<<16),
		/* G */			MTX_INIT_2X2(KF_TACHO_SYS_RI,     1<<16,  0, 								  0, 1<<16),
		/* Q */			MTX_INIT_2X2(KF_TACHO_SYS_RI,     10<<16, 0, 								  0, 2500<<16)
					},
	/* MeasFcts */KF_MeasValFctHdlsRi,
	/* InptFcts */NULL,
	/*Modulo counter*/TRUE
				},
/* Data */		KF_DFLT_DATA_INIT(KF_TACHO_SYS_RI)
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
