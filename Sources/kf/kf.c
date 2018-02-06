/*************************************************************************************************
 * @file		kf.c
 * @ingroup		kf Kalman FIlter
 * @brief 		This Module implements a Kalman Filter.
 *
 *
 *
 * @author  G. Freudenthaler, gfre@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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
static KF_MtxCfg_t *KF_MtxCfg = NULL;
static KF_DimCfg_t *KF_Dim    = NULL;
static KF_Cfg_t    *KF_Cfg   = NULL;
static KF_Data_t   *KF_Data   = NULL;
static MTX_t KF_mEye = { FIXMATRIX_MAX_SIZE, FIXMATRIX_MAX_SIZE, 0, {{1<<16,0,0,0},
																	{0,1<<16,0,0},
																	{0,0,1<<16,0},
																	{0,0,0,1<<16}} };


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_)
{
	uint8_t i = 0u;
	int32_t tmp = 0;
	if(NULL != kf_)
	{
//		MTX_FillDiagonal( &kf_->data.mDPapost, fix16_from_int(100) );
//		MTX_FillDiagonal( &kf_->data.mUPapost, fix16_one);
		kf_->data.nMdCntr = 0;
		for(i = 0u; i < kf_->cfg.dim.nMsrdSts; i++)
		{
			kf_->cfg.aMeasValFct[i]( &tmp );
			kf_->data.vXapost.data[i][0] = fix16_from_int(400);
		}
	}
	else
	{
		/* error handling */
	}
}

//static StdRtn_t KF_BiermanObservationalUpdate(const int32_t yj_, const int32_t rjj_, const MTX_t *vCjT_, MTX_t *vX_post_, MTX_t *mUP_post_, MTX_t *mDP_post_)
//{
//	StdRtn_t retVal = ERR_PARAM_ADDRESS;
//	uint8_t i = 0u, j = 0u;
//	int32_t delta = 0;
//	int32_t sigma = 0, nu = 0, tau = 0, epsilon = 0;
//	MTX_t vV = { V, dim_.nSys, 1, {0} };
//	MTX_t vW = { W, dim_.nSys, 1, {0} };
//
//	if( (NULL != vX_post_) && (NULL != mUP_post_) && (NULL != mDP_post_) )
//	{
//		retVal = ERR_OK;
//		delta = yj_;
//		for(j = 0u; j < dim_.nMsrdSts; j++)
//		{
//			delta = delta - MTXLOC_ij(vCjT_->mtx, 0, j) * MTXLOC_ij(vX_post_->mtx, j, 0);
//			MTXLOC_ij(vV, j, 0) = MTXLOC_ij(vCjT_->mtx, 0, j);
//			for(i = 0u; i < (j-1); i++)
//			{
//				MTXLOC_ij(vV, j, 0) = MTXLOC_ij(vV, j, 0) + MTXLOC_ij(mUP_post_->mtx, i, j) * MTXLOC_ij(vCjT_->mtx, 0, i);
//			}
//		}
//		sigma = rjj_;
//		for(j = 0u; j < dim_.nMsrdSts; j++)
//		{
//			nu = MTXLOC_ij(vV, j, 0);
//			MTXLOC_ij(vV, j, 0) = MTXLOC_ij(vV, j, 0) * MTXLOC_ij(mDP_post_->mtx, j , j);
//			MTXLOC_ij(vW, j, 0) = nu;
//			for(i = 0u; i < (j-1); i++)
//			{
//				tau = MTXLOC_ij(mUP_post_->mtx, i, j) * nu;
//				MTXLOC_ij(mUP_post_->mtx, i, j) = MTXLOC_ij(mUP_post_->mtx, i, j) - (nu * MTXLOC_ij(vW, i, 0)) / sigma;
//				MTXLOC_ij(vW, i, 0) = MTXLOC_ij(vW, i, 0) + tau;
//			}
//			MTXLOC_ij(mDP_post_->mtx, j, j) = MTXLOC_ij(mDP_post_->mtx, j, j) * sigma;
//		}
//		epsilon = delta / sigma;
//		for(i = 0u; i < dim_.nSys; i++)
//		{
//			MTXLOC_ij(vX_post_->mtx, i, 0) = MTXLOC_ij(vX_post_->mtx, i, 0) + MTXLOC_ij(vV, i, 0) * epsilon;
//		}
//	}
//	return retVal;
//}


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
	MTX_t mTmp1 = {0}, mTmp2 = {0}, mTmp3 = {0};
	int32_t yj     = 0;

	if( (NULL != KF_pTbl) && (NULL != KF_pTbl->aKfs) )
	{
		for(i = 0u; i < KF_pTbl->numKfs; i++)
		{
			/* Update matrix dimension information for current KF */
			KF_MtxCfg = &KF_pTbl->aKfs[i].cfg.mtx;
			KF_Dim    = &KF_pTbl->aKfs[i].cfg.dim;
			KF_Data   = &KF_pTbl->aKfs[i].data;
			KF_Cfg    = &(KF_pTbl->aKfs[i].cfg);
			KF_mEye.rows     = KF_Dim->nSys;
			KF_mEye.columns  = KF_Dim->nSys;

			/**
			 * Temporal update
			 */
				/* x_{k}(-) */
				MTX_Mult( &(KF_Data->vXapri), &(KF_MtxCfg->mSys), &(KF_Data->vXapost) );
				/* P_{k}(-) using modified weighted Gram-Schmidt-Orthonogonalization according to C. Thornton */
					/* A = [U_{k-1}(+)'Phi'; (G*UQ)'] */
				MTX_Transpose( &mTmp1, &(KF_Data->mUPapost) ); //, tmp1 = UPapost'
				MTX_MultBt( &mTmp1, &mTmp1, &(KF_MtxCfg->mSys) );  // tmp1 = UPapost'*Phi'
				MTX_AppendMatrix( &mTmp2, &mTmp1, &KF_mEye, (mTmp1.rows+1), 1); // tmp2  = A
					/* A = BL -> QL-Decomposition, where L' = U_{k}(-), ... */
				MTX_QlDecomposition(&mTmp2, &mTmp1, &mTmp2, 0);  // tmp2 = B, tmp1 = L = U_{k}(-)'
					/* ... and B'DwB = D_{k}(-), with Dw = [DP_{k-1}(+) 0; 0 DQ] */
				MTX_AppendMatrix(&mTmp3, &(KF_Data->mDPapost), &(KF_MtxCfg->mPrcsNsCov), (KF_Data->mDPapost.rows+1), (KF_Data->mDPapost.columns+1)); // tmp3 = Db
				MTX_MultAt(&mTmp3, &mTmp2, &mTmp3); // tmp3 = B'Db
				MTX_Mult(&mTmp3, &mTmp3, &mTmp2);   // tmp3 = DP_{k}(-)
			/**
			 * Measurement update
			 */
				KF_Data->vXapost  = KF_Data->vXapri;
				MTX_Transpose(&KF_Data->mUPapost, &mTmp1);
				KF_Data->mDPapost = mTmp3;
				for(j = 0; j < KF_Dim->nMsrdSts; j++)
				{
					KF_Cfg->aMeasValFct[j](&yj);
//					KF_BiermanObservationalUpdate(yj, KF_MtxCfg.mMeasNsCov.data[j][j], &(KF_MtxCfg.mMeas.data[j][0]), &KF_Data.vXapost, &KF_Data.mUPapost, &KF_Data.mDPapost);
				}
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
			*pVal_ = KF_pTbl->aKfs[idx_].data.vXapost.data[2][0];
			retVal = ERR_OK;
		}
	}
	return retVal;
}



#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
