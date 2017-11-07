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
 *
 */
typedef struct KF_SclCfg_s
{
	const uint8_t nSysMtx;
	const uint8_t nErrMtx;
	const uint8_t nStVec;
	const int32_t nMxMdloVal;
}KF_SclCfg_t;

/**
 * runtime data
 */
typedef struct KF_Data_s
{
	MTX_t  vPrvStEst;
	MTX_t  mPrvErrCoVar;
	int8_t nMdCntr;
}KF_Data_t;

/**
 *
 */
typedef struct KF_MtxCfg_s
{
	const MTX_t mSys;
	const MTX_t mSysTrnsp;
	const MTX_t mInpt;
	const MTX_t mMeas;
	const MTX_t mMeasTrnsp;
	const MTX_t mMeasNsCov;
	const MTX_t mPrcsNsCov;
}KF_MtxCfg_t;

/**
 *
 */
typedef struct KF_DimCfg_s
{
	const uint8_t nInpts;
	const uint8_t nMsrdSts;
}KF_DimCfg_t;

/**
 *
 */
typedef struct KF_Cfg_s
{
	const uchar_t *pItmName;
	const uint8_t smplTimeMS;
	const KF_MtxCfg_t *mtx;
	const KF_SclCfg_t *scl;
	const KF_DimCfg_t *dim;
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
	KF_Itm_t *aTls;
	const uint8_t numTls;
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
