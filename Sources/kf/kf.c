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
static StdRtn_t KF_Predict(KF_Itm_t *kf_);
static StdRtn_t KF_Predict_x(KF_Itm_t *kf_);
static StdRtn_t KF_Predict_P(KF_Itm_t *kf_);
static StdRtn_t KF_BiermanObservationalUpdate(MTX_t *vXpost_, MTX_t *mUPpost_, MTX_t *mDPpost_, const int32_t yj_, const int32_t rjj_, const MTX_t *mC_, const uint8_t row_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static KF_ItmTbl_t *KF_pTbl  = NULL;



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

static StdRtn_t KF_Predict(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != kf_)
	{
		retVal  = ERR_OK;
		retVal |= KF_Predict_x(kf_);
		retVal |= KF_Predict_P(kf_);
	}
	return retVal;
}

static StdRtn_t KF_Predict_x(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != kf_)
	{
		retVal = ERR_OK;
		MTX_Mult(&kf_->data.vXapri, &kf_->cfg.mtx.mSys, &kf_->data.vXapost);
	}
	return retVal;
}

/* TODO change tmp2 to G_dash... maybe in config? Also, overflow handling*/
static StdRtn_t KF_Predict_P(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int8_t i = 0, j = 0, k = 0, dim = 0;
	MTX_t tmp = {0}, tmp2 = {0};
	int32_t sigma = 0;
	if(NULL != kf_)
	{
		retVal = ERR_OK;
		dim = kf_->cfg.dim.nSys;
		tmp2.rows = dim;
		tmp2.columns = dim;
		MTX_FillDiagonal(&tmp2, fix16_one); /* tmp2 = G_dash */
		MTX_Mult(&tmp, &kf_->cfg.mtx.mSys, &kf_->data.mUPapost); /* tmp = PhiU */
		for(i = (dim-1); i >= 0; i--)
		{
			sigma = 0;
			for(j = 0; j < dim; j++)
			{
				sigma = fix16_add(sigma, fix16_mul(fix16_sq(tmp.data[i][j]), kf_->data.mDPapost.data[j][j]));
				if(j <= (dim-1))
				{
					sigma = fix16_add(sigma, fix16_mul(fix16_sq(tmp2.data[i][j]), kf_->cfg.mtx.mPrcsNsCov.data[j][j]));
				}
			}
			kf_->data.mDPapri.data[i][i] = sigma;
			kf_->data.mUPapri.data[i][i] = fix16_one;
			for(j = 0; j <= (i-1); j++)
			{
				sigma = 0;
				for(k = 0; k < (dim); k++)
				{
					sigma = fix16_add(sigma, fix16_mul(tmp.data[i][k], fix16_mul(kf_->data.mDPapost.data[k][k], tmp.data[j][k])));
				}
				for(k = 0; k < (dim); k++)
				{
					sigma = fix16_add(sigma, fix16_mul(tmp2.data[i][k], fix16_mul(kf_->cfg.mtx.mPrcsNsCov.data[k][k], tmp2.data[j][k])));
				}
				kf_->data.mUPapri.data[j][i] = fix16_div(sigma, kf_->data.mDPapri.data[i][i]);
				for(k = 0; k < (dim); k++)
				{
					tmp.data[j][k] = fix16_sub(tmp.data[j][k], fix16_mul(kf_->data.mUPapri.data[j][i], tmp.data[i][k]));
				}
				for(k = 0; k < (dim); k++)
				{
					tmp2.data[j][k] = fix16_sub(tmp2.data[j][k], fix16_mul(kf_->data.mUPapri.data[j][i], tmp2.data[i][k]));
				}
			}
		}
	}
	return retVal;
}

static StdRtn_t KF_BiermanObservationalUpdate(MTX_t *vXpost_, MTX_t *mUPpost_, MTX_t *mDPpost_, const int32_t yj_, const int32_t rjj_, const MTX_t *mC_, const uint8_t row_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u, j = 0u;
	int32_t delta = 0, sigma = 0, nu = 0, tau = 0, epsilon = 0;
	int32_t vV[vXpost_->rows], vW[vXpost_->rows];

	if( (NULL != vXpost_) && (NULL != mUPpost_) && (NULL != mDPpost_) )
	{
		retVal = ERR_OK;
		delta = yj_;
		for(j = 0u; j < vXpost_->rows; j++)
		{
			delta = delta - fix16_mul(mC_->data[row_][j], vXpost_->data[j][0]);
			vV[j]  = mC_->data[row_][j];
			for(i = 0u; i < (j-1); i++)
			{
				vV[j] = fix16_add(vV[j], fix16_mul(mUPpost_->data[i][j], mC_->data[row_][i]));
			}
		}
		sigma = rjj_;
		for(j = 0u; j < vXpost_->rows; j++)
		{
			nu = vV[j];
			vV[j] = fix16_mul(vV[j], mDPpost_->data[j][j]);
			vW[j] = nu;
			for(i = 0u; i < (j-1); i++)
			{
				tau = fix16_mul(mUPpost_->data[i][j], nu);
				mUPpost_->data[i][j] = fix16_div(fix16_sub(mUPpost_->data[i][j], fix16_mul(nu, vW[i])), sigma);
				vW[i] = fix16_add(vW[i], tau);
			}
			mDPpost_->data[j][j] = fix16_mul(mDPpost_->data[j][j], sigma);
		}
		epsilon = fix16_div(delta, sigma);
		for(i = 0u; i < vXpost_->rows; i++)
		{
			vXpost_->data[i][0] = fix16_add(vXpost_->data[i][0], fix16_mul(vV[i], epsilon));
		}
	}
	return retVal;
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
	uint8_t i = 0u, j = 0u;
	int32_t yj = 0;

	if( (NULL != KF_pTbl) && (NULL != KF_pTbl->aKfs) )
	{
		for(i = 0u; i < KF_pTbl->numKfs; i++)
		{
			KF_Predict(&KF_pTbl->aKfs[i]);

//			KF_Data->vXapost  = KF_Data->vXapri;
//			MTX_Transpose(&KF_Data->mUPapost, &mTmp1);
//			KF_Data->mDPapost = mTmp3;
//			for(j = 0; j < KF_Dim->nMsrdSts; j++)
//			{
//				KF_Cfg->aMeasValFct[j](&yj);
//				KF_BiermanObservationalUpdate(&KF_Data.vXapost, &KF_Data.mUPapost, &KF_Data.mDPapost, yj, KF_MtxCfg.mMeasNsCov.data[j][j], &(KF_MtxCfg.mMeas.data[j][0]), j);
//			}
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
