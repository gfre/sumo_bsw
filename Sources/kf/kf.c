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
#define KF_UPSCALE(val_, nScl_) (val_<<nScl_)
#define KF_DOWNSCALE(val_, nScl_) (val_>>nScl_)

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
KF_ItmTbl_t *KF_pTbl = NULL;

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_)
{
	uint8_t i = 0u;
	if(NULL != kf_)
	{
		MTX_FillDiag( &(kf_->data->mPrvErrCoVar), (uint8_t)100 );
		MTX_ScaleUp( &(kf_->data->mPrvErrCoVar), kf_->cfg.scl->nErrMtx );
		kf_->data->nMdCntr = 0;
		for(i = 0u; i < kf_->cfg.dim->nMsrdSts; i++)
		{
			kf_->cfg.aMeasValFct[i]( &(MTXLOC_ij( (kf_->data->vPrvStEst), i, 0) ) );
		}
		MTX_ScaleUp( &(kf_->data->vPrvStEst), kf_->cfg.scl->nStVec);
	}
	else
	{
		/* error handling */
	}
}

static void KF_BiermanObservationUpdate( int32_t yj_, int32_t rjj_, MTX_t *cj_, MTX_t *mtxu_, MTX_t *mtxd_, MTX_t *x_, KF_Cfg_t *pCfg_)
{
	uint8_t i = 0u, j = 0u;
	int32_t delta = yj_;
	int32_t CjT_m_xj[1][1];

	MTX_t sCjT_m_cj = { CjT_m_xj[0], 1, 1 };

	for(j = 0u; j < pCfg_->dim->nMsrdSts; j++)
	{
		MTX_Mult(cj_, x_, &sCjT_m_cj);
		delta -= CjT_m_xj[1][1];
	}

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
			KF_Data_t *pData = KF_pTbl->aKfs[i].data;
			KF_Cfg_t  *pCfg  = &(KF_pTbl->aKfs[i].cfg);

			/**
			 *  temporarily used matrices/vectors for prediction
			 */
			int32_t Axkk[pCfg->dim->nSys][1];
			int32_t uk[pCfg->dim->nInpts][1];
			int32_t Buk[pCfg->dim->nMsrdSts][1];
			int32_t xk1k[pCfg->dim->nSys][1];
			int32_t APkk[pCfg->dim->nSys][pCfg->dim->nSys];
			int32_t APkkAT[pCfg->dim->nSys][pCfg->dim->nSys];
			int32_t Pk1k[pCfg->dim->nSys][pCfg->dim->nSys];

			MTX_t vAxkk   = { Axkk[0],   pCfg->dim->nSys,   1 };
			MTX_t vUk 	  = { uk[0],     pCfg->dim->nInpts, 1 };
			MTX_t vBuk 	  = { Buk[0],    pCfg->dim->nSys,   1 };
			MTX_t vXk1k	  = { xk1k[0],   pCfg->dim->nSys,   1 };
			MTX_t mAPkk   = { APkk[0],   pCfg->dim->nSys,   pCfg->dim->nSys};
			MTX_t mAPkkAT = { APkkAT[0], pCfg->dim->nSys,   pCfg->dim->nSys};
			MTX_t mPk1k   = { Pk1k[0],   pCfg->dim->nSys,   pCfg->dim->nSys};

			/**
			 * time update / 'predict'
			 */

			// x(k+1|k)
			MTX_Mult( &(pCfg->mtx->mSys), &(pData->vPrvStEst), &(vAxkk));
			MTX_ScaleDown( &(vAxkk), pCfg->scl->nSysMtx);

			for(j = 0u; j < pCfg->dim->nInpts; j++)
			{
				if(NULL != pCfg->aInptValFct[j])
				{
					pCfg->aInptValFct[j]( &(MTXLOC_ij(vUk, j, 0) ) );
				}
				else
				{
					/* error handling */
				}
			}

			MTX_Mult( &(pCfg->mtx->mInpt), &(vUk), &(vBuk) );
			MTX_ScaleUp( &(vBuk), pCfg->scl->nStVec );
			MTX_Add( &(vAxkk), &(vBuk), &(vXk1k) );

			//P(k+1|k)
			MTX_Mult( &(pCfg->mtx->mSys), &(pData->mPrvErrCoVar), &(mAPkk) );
			MTX_ScaleDown( &(mAPkk), pCfg->scl->nSysMtx );
			MTX_Mult( &(mAPkk), &(pCfg->mtx->mSysTrnsp), &(mAPkkAT) );
			MTX_ScaleDown( &(mAPkkAT), pCfg->scl->nSysMtx );
			MTX_Add( &(mAPkkAT), &(pCfg->mtx->mPrcsNsCov), &(mPk1k) );


			/**
			 *  bierman observational update according to "Mohinder S. Grewal, Angus P. Andrews: Kalman Filtering
			 *  Theory and Practice using MATLAB, 3rd Edition, 2008, p270 table 6.15"
			 */
			int32_t U[pCfg->dim->nMsrdSts][pCfg->dim->nMsrdSts];
			int32_t D[pCfg->dim->nMsrdSts][pCfg->dim->nMsrdSts];
			int32_t cjT[1][pCfg->dim->nMsrdSts];
			int32_t yj;
			int32_t cjT_m_xj;

			MTX_t mU	= { U[0],  pCfg->dim->nMsrdSts, pCfg->dim->nMsrdSts };
			MTX_t mD	= { D[0],  pCfg->dim->nMsrdSts, pCfg->dim->nMsrdSts };
			MTX_t vCjT	= { cjT[0], 1,					pCfg->dim->nMsrdSts };


			MTX_UdDecomp(&mPk1k, &mU, &mD, 10);

			for(k = 0u; k < pCfg->dim->nMsrdSts; k++)
			{
				pCfg->aMeasValFct[k]( &yj );
				KF_BiermanObservationUpdate( yj, MTX_ij(pCfg->mtx->mMeasNsCov, k, k), &vCjT, &mU, &mD, &vXk1k, pCfg);
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
			*pVal_ = MTXLOC_ij( KF_pTbl->aKfs[idx_].data->vOptStEst, idx_, 0 );
			retVal = ERR_OK;
		}
	}
	return retVal;
}



#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
