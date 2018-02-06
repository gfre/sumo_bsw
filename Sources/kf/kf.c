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
#include "fixarray.h"


/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_);
static StdRtn_t KF_Predict_x(KF_Itm_t *kf_);
static StdRtn_t KF_Predict_P(KF_Itm_t *kf_);
static StdRtn_t KF_Correct(KF_Itm_t *kf_);
static inline StdRtn_t KF_ThorntonTemporalUpdate(MTX_t *mUPapri_, MTX_t *mDPapri_, const MTX_t *Phi_, const MTX_t *mUPapost_, const MTX_t *mDPapost_, MTX_t *mGUQ_, const MTX_t *mDQ_);
static StdRtn_t KF_BiermanObservationalUpdate(MTX_t *vXapost_, MTX_t *mUPapost_, MTX_t *mDPapost_, const int32_t yj_, const int32_t rjj_, const MTX_t *mC_, const uint8_t row_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static KF_ItmTbl_t *KF_pTbl  = NULL;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_)
{
	uint8_t i = 0u;
	int32_t tmp = 0;
	if(NULL != kf_)
	{
/* commented for debugging purposes. look at kf_config */
//		MTX_FillDiagonal( &kf_->data.mDPapost, fix16_from_int(100) );
//		MTX_FillDiagonal( &kf_->data.mUPapost, fix16_one);
		kf_->data.nMdCntr = 0;
		for(i = 0u; i < kf_->cfg.dim.nMsrdSts; i++)
		{
			kf_->cfg.aMeasValFct[i]( &tmp );
			kf_->data.vXapost.data[i][0] = fix16_from_int(400); /* bot initial states set to 400 for debugging purposes */
		}
	}
}

static StdRtn_t KF_Predict_x(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != kf_)
	{
		retVal = ERR_OK;
		MTX_Mult(&(kf_->data.vXapri), &(kf_->cfg.mtx.mSys), &(kf_->data.vXapost));
	}
	return retVal;
}

static StdRtn_t KF_Predict_P(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	MTX_t mGUQ = {0};

	if( NULL != kf_ )
	{
		retVal       = ERR_OK;
		mGUQ.rows    = kf_->cfg.dim.nSys; /* TODO G and UQ respectively part of config?! */
		mGUQ.columns = kf_->cfg.dim.nSys;
		MTX_FillDiagonal(&mGUQ, fix16_one);
		retVal |= KF_ThorntonTemporalUpdate(&(kf_->data.mUPapri), &(kf_->data.mDPapri), &(kf_->cfg.mtx.mSys), &(kf_->data.mUPapost), &(kf_->data.mDPapost), &(mGUQ), &(kf_->cfg.mtx.mPrcsNsCov));
	}
	return retVal;
}

static StdRtn_t KF_Correct(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t m = 0u;
	int32_t ym[2] = {401<<16, 500<<16}; /* set for debugging purposes */
	if( NULL != kf_ )
	{
		retVal             = ERR_OK;
		kf_->data.vXapost  = kf_->data.vXapri;
		kf_->data.mUPapost = kf_->data.mUPapri;
		kf_->data.mDPapost = kf_->data.mDPapri;
		for(m = 0; m < kf_->cfg.dim.nMsrdSts; m++)
		{
			/* commented for debugging purposes */
			//kf_->cfg.aMeasValFct[m](&ym);
			retVal |= KF_BiermanObservationalUpdate(&(kf_->data.vXapost), &(kf_->data.mUPapost), &(kf_->data.mDPapost), ym[m], kf_->cfg.mtx.mMeasNsCov.data[m][m], &(kf_->cfg.mtx.mMeas), m);
		}
		/* next 3 lines for debugging purposes */
		MTX_t tmp = {0};
		MTX_Mult(&tmp, &(kf_->data.mUPapost), &(kf_->data.mDPapost));
		MTX_MultBt(&tmp, &tmp, &(kf_->data.mUPapost)); /* tmp = P_apost -> not positive definite!!!! */
	}
}

/* TODO overflow handling */
static inline StdRtn_t KF_ThorntonTemporalUpdate(MTX_t *mUPapri_, MTX_t *mDPapri_, const MTX_t *Phi_, const MTX_t *mUPapost_, const MTX_t *mDPapost_, MTX_t *mGUQ_, const MTX_t *mDQ_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int8_t i = 0;
	uint8_t j = 0u, k = 0u, dim = 0u;
	MTX_t tmp = {0};
	int32_t sigma = 0;

	if( (NULL != mUPapri_) && (NULL != mDPapri_) && (NULL != mGUQ_) )
	{
		retVal = ERR_OK;
		dim = Phi_->rows;
		MTX_Mult(&tmp, Phi_, mUPapost_); /* tmp = PhiU */
		for(i = (dim-1); i >= 0; i--)
		{
			sigma = 0;
			for(j = 0; j < dim; j++)
			{
				sigma = fix16_add(sigma, fix16_mul(fix16_sq(tmp.data[i][j]), mDPapost_->data[j][j]));
				if(j <= (dim-1))
				{
					sigma = fix16_add(sigma, fix16_mul(fix16_sq(mGUQ_->data[i][j]), mDQ_->data[j][j]));
				}
			}
			mDPapri_->data[i][i] = sigma;
			mUPapri_->data[i][i] = fix16_one;
			for(j = 0; j <= (i-1); j++)
			{
				sigma = 0;
				for(k = 0; k < (dim); k++)
				{
					sigma = fix16_add(sigma, fix16_mul(tmp.data[i][k], fix16_mul(mDPapost_->data[k][k], tmp.data[j][k])));
				}
				for(k = 0; k < (dim); k++)
				{
					sigma = fix16_add(sigma, fix16_mul(mGUQ_->data[i][k], fix16_mul(mDQ_->data[k][k], mGUQ_->data[j][k])));
				}
				mUPapri_->data[j][i] = fix16_div(sigma, mDPapri_->data[i][i]);
				for(k = 0; k < (dim); k++)
				{
					tmp.data[j][k] = fix16_sub(tmp.data[j][k], fix16_mul(mUPapri_->data[j][i], tmp.data[i][k]));
				}
				for(k = 0; k < (dim); k++)
				{
					mGUQ_->data[j][k] = fix16_sub(mGUQ_->data[j][k], fix16_mul(mUPapri_->data[j][i], mGUQ_->data[i][k]));
				}
			}
		}
	}
	return retVal;
}

/* TODO overflow handling */
static StdRtn_t KF_BiermanObservationalUpdate(MTX_t *vXapost_, MTX_t *mUPapost_, MTX_t *mDPapost_, const int32_t y_, const int32_t r_, const MTX_t *mC_, const uint8_t currRow_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u, j = 0u;
	int32_t dz = 0, alpha = 0, gamma = 0, beta = 0, lambda = 0;
	int32_t a[vXapost_->rows], b[vXapost_->rows];

	if( (NULL != vXapost_) && (NULL != mUPapost_) && (NULL != mDPapost_) )
	{
		retVal = ERR_OK;
		for(i = 0u; i < mUPapost_->rows; i++)
		{
			a[i] = fa16_dot(&(mUPapost_->data[0][i]), FIXMATRIX_MAX_SIZE, &(mC_->data[currRow_][0]), 1, mUPapost_->rows);
		}
		for(i = 0u; i < mUPapost_->rows; i++)
		{
			b[i] = fa16_dot(&(mDPapost_->data[i][0]), 1, a, 1, mDPapost_->rows);
		}
		dz = fix16_sub(y_, fa16_dot( &(mC_->data[currRow_][0]), 1, &(vXapost_->data[0][0]), FIXMATRIX_MAX_SIZE, vXapost_->rows) );
		alpha = r_;
		gamma = fix16_div(fix16_one, alpha);
		for(j = 0u; j < vXapost_->rows; j++)
		{
			beta   = alpha;
			alpha  = fix16_add( alpha, fix16_mul(a[j], b[j]) );
			lambda = fix16_mul( (-a[j]), gamma );
			gamma  = fix16_div( fix16_one, alpha );
			mDPapost_->data[j][j] = fix16_mul( beta, fix16_mul(gamma, mDPapost_->data[j][j]) );
			for(i = 0u; i < j; i++)
			{
				beta = mUPapost_->data[i][j];
				mUPapost_->data[i][j] = fix16_add( beta, fix16_mul(b[i], lambda) ); /* TODO shouldn't be negative!! */
				b[i] = fix16_add( b[i], fix16_mul(b[j], beta) );
			}
		}
		for(i = 0; i < vXapost_->rows; i++)
		{
			vXapost_->data[i][0] = fix16_add( vXapost_->data[i][0], fix16_mul(fix16_mul(gamma, dz), b[i]) );
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
}

void KF_Main(void)
{
	uint8_t i = 0u;
	if( (NULL != KF_pTbl) && (NULL != KF_pTbl->aKfs) )
	{
		for(i = 0u; i < KF_pTbl->numKfs; i++)
		{
			KF_Predict_x(&KF_pTbl->aKfs[i]);
			KF_Predict_P(&KF_pTbl->aKfs[i]);
			KF_Correct(&KF_pTbl->aKfs[i]);
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
