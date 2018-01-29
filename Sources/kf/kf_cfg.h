/***********************************************************************************************//**
 * @file		kf_cfg.h
 * @ingroup		kf Kalman Filter
 * @brief 		This header file contains the type definitions and type configurations for matrices,
 * 				column and row vectors for the Kalman-Filter.
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	26.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef KF_CFG_H
#define KF_CFG_H

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"
#include "Acon_Types.h"
#include "mtx_api.h"


#ifdef MASTER_KF_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef StdRtn_t (*KF_ReadFct_t)(int32_t*);

/**
 * runtime data
 */
typedef struct KF_Data_s
{
	MTX_t  vPrvStEst;
	MTX_t  vOptStEst;
	MTX_t  mPrvUP;
	MTX_t  mPrvDP;
	int8_t nMdCntr;
}KF_Data_t;

/**
 *
 */
typedef struct KF_MtxCfg_s
{
	 MTX_t mSys;
	 MTX_t mInpt;
	 MTX_t mMeas;
	 MTX_t mMeasTrnsp;
	 MTX_t mMeasNsCov;
	 MTX_t mPrcsNsCov;
}KF_MtxCfg_t;

/**
 *
 */
typedef struct KF_DimCfg_s
{
	 uint8_t nSys;
	 uint8_t nInpts;
	 uint8_t nMsrdSts;
}KF_DimCfg_t;

/**
 *
 */
typedef struct KF_Cfg_s
{
	const uchar_t *pItmName;
	const uint8_t smplTimeMS;
	KF_MtxCfg_t mtx;
	KF_DimCfg_t dim;
	KF_ReadFct_t *aMeasValFct;
	KF_ReadFct_t *aInptValFct;
}KF_Cfg_t;

/**
 *
 */
typedef struct KF_Itm_s
{
	KF_Cfg_t cfg;
	KF_Data_t data;
}KF_Itm_t;

/**
 *
 */
typedef struct KF_ItmTbl_s
{
	KF_Itm_t *aKfs;
	const uint8_t numKfs;
} KF_ItmTbl_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ KF_ItmTbl_t *Get_pKfItmTbl(void);



/**
 * wrapper functions to fit data type for KF_ReadFct_t
 */
EXTERNAL_ StdRtn_t KF_Read_Rawi32SpdLe(int32_t *spd_);
EXTERNAL_ StdRtn_t KF_Read_Rawi32SpdRi(int32_t *spd_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* KF_CFG_H */
