/*************************************************************************************************
 * @file		kf.c
 * @ingroup		kf Kalman FIlter
 * @brief 		This Module implements a Kalman Filter.
 *
 *
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
#include "kf_api.h"


/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static KF_ItmTbl_t *KF_pTbl  = NULL;
static KF_MtxCfg_t KF_MtxCfg = {0};
static KF_DimCfg_t KF_Dim    = {0};
static KF_Data_t   *KF_Data  = NULL;
static MTX_t KF_mEye = {FIXMATRIX_MAX_SIZE, FIXMATRIX_MAX_SIZE, 0, {{1<<16,0,0,0,0,0,0,0},
																	{0,1<<16,0,0,0,0,0,0},
																	{0,0,1<<16,0,0,0,0,0},
																	{0,0,0,1<<16,0,0,0,0},
																	{0,0,0,0,1<<16,0,0,0},
																	{0,0,0,0,0,1<<16,0,0},
																	{0,0,0,0,0,0,1<<16,0},
																	{0,0,0,0,0,0,0,1<<16}}};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_)
{
	uint8_t i = 0u;
	int32_t tmp = 0;
	if(NULL != kf_)
	{
		MTX_FillDiagonal( &kf_->data.mPrvDP, fix16_from_int(100) );
		MTX_FillDiagonal( &kf_->data.mPrvUP, fix16_one);
		kf_->data.nMdCntr = 0;
		for(i = 0u; i < kf_->cfg.dim.nMsrdSts; i++)
		{
			kf_->cfg.aMeasValFct[i]( &tmp );
			kf_->data.vPrvStEst.data[i][0] = fix16_from_int(400);
		}
	}
	else
	{
		/* error handling */
	}
}

static void KF_BiermanObservationUpdate( int32_t yj_, int32_t rjj_, MTX_t *cj_, MTX_t *mtxu_, MTX_t *mtxd_, MTX_t *x_, KF_Cfg_t *pCfg_)
{


}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void KF_Init(void)
{
	uint8_t i = 0u;
	KF_pTbl = Get_pKfItmTbl();
	if( (NULL != KF_pTbl) && (NULL != KF_pTbl->aKfs) )
	{
		for(i = 0u; i < KF_pTbl->numKfs; i++)
		{
			KF_Reset( &(KF_pTbl->aKfs[i]) );
		}
	}
	else
	{
		/* error handling */
	}
}


void KF_Main(void)
{
	StdRtn_t retVal = ERR_OK;
	uint8_t i = 0u, j = 0u, k = 0u;

	if( (NULL != KF_pTbl) && (NULL != KF_pTbl->aKfs) )
	{
		for(i = 0u; i < KF_pTbl->numKfs; i++)
		{
			/* Update matrix dimension information for current KF */
			KF_MtxCfg = KF_pTbl->aKfs[i].cfg.mtx;
			KF_Dim    = KF_pTbl->aKfs[i].cfg.dim;
			KF_Data   = &(KF_pTbl->aKfs[i].data);
			KF_mEye.rows    = KF_Dim.nSys;
			KF_mEye.columns = KF_Dim.nSys;

			/* temporarily used matrices/vectors */
			MTX_t vPhixkk  = { KF_Dim.nMsrdSts, 1,                      			 0, {0} };
			MTX_t mUPT	   = { KF_Dim.nMsrdSts, KF_Dim.nMsrdSts,        			 0, {0} };
			MTX_t mUPTPhiT = { KF_Dim.nMsrdSts, KF_Dim.nMsrdSts,         			 0, {0} };
			MTX_t mA       = { KF_Dim.nMsrdSts+KF_Dim.nSys, KF_Dim.nSys,             0, {0} };
			MTX_t mDw	   = { KF_Dim.nMsrdSts+KF_Dim.nSys, KF_Dim.nSys+KF_Dim.nSys, 0, {0} };
			MTX_t mB	   = { KF_Dim.nMsrdSts+KF_Dim.nSys, KF_Dim.nSys,             0, {0} };
			MTX_t mL       = { KF_Dim.nMsrdSts, KF_Dim.nMsrdSts,         			 0, {0} };

			/**
			 * Temporal update
			 */
				/* x(k+1|k) */
				MTX_Mult( &(vPhixkk), &(KF_MtxCfg.mSys), &(KF_Data->vPrvStEst) );
				/* P(k+1|k) using modified weighted Gram-Schmidt-Orthonogonalization according to C. Thornton */
				MTX_Transpose( &mUPT, &(KF_Data->mPrvUP) );
				MTX_MultBt( &mUPTPhiT, &mUPT, &(KF_MtxCfg.mSys) );
				MTX_AppendRow( &mA, &mUPTPhiT, &KF_mEye );
				MTX_QlDecomposition(&mB, &mL, &mA, 0);

			/**
			 * Measurement update
			 */
		}
	}
}

void KF_Deinit(void)
{
	KF_pTbl = NULL;
}


StdRtn_t KF_Read_i16EstdVal(int16_t *pVal_, const uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( (NULL != pVal_) && (NULL != KF_pTbl) && (NULL != KF_pTbl->aKfs) )
	{
		retVal = ERR_PARAM_VALUE;
		if(idx_ < KF_pTbl->numKfs )
		{
			*pVal_ = KF_pTbl->aKfs[idx_].data.vOptStEst.data[2][0];
			retVal = ERR_OK;
		}
	}
	return retVal;
}



#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
