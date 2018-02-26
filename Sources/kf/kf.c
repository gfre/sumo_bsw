/*************************************************************************************************
 * @file		kf.c
 * @ingroup		kf Kalman FIlter
 * @brief 		This Module implements a Kalman Filter.
 *
 *
 *
 * @author  G. Freudenthaler, gfre@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author 	S. Helling, stu112498@tf.uni-kiel.de,  Chair of Automatic Control, University Kiel
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
#include "fixarray.h" /* dot product in Bierman update */


/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_);
int64_t KF_48d16Dot(const fix16_t *a, uint_fast8_t a_stride, const fix16_t *b, uint_fast8_t b_stride, uint_fast8_t n);
static StdRtn_t KF_UpdateModuloCounter(KF_Data_t *data_);
static StdRtn_t KF_Predict_x(KF_Itm_t *kf_);
static StdRtn_t KF_Predict_P(KF_Itm_t *kf_);
static StdRtn_t KF_Correct(KF_Itm_t *kf_);
static StdRtn_t KF_ThorntonTemporalUpdate(MTX_t *mUPapri_, MTX_t *mDPapri_, const MTX_t *Phi_, const MTX_t *mUPapost_, const MTX_t *mDPapost_, MTX_t *mGUQ_, const MTX_t *mDQ_);
static StdRtn_t KF_BiermanObservationalUpdate(MTX_t *vXapost_, MTX_t *mUPapost_, MTX_t *mDPapost_, int32_t dym_, int32_t rmm_, const MTX_t *mH_, uint8_t m_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static KF_ItmTbl_t *KF_pTbl  = NULL;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void KF_Reset(KF_Itm_t *kf_)
{
	uint8_t i = 0u;
	int32_t tmp = 0;
	if(NULL != kf_)
	{
		MTX_FillDiagonal( &(kf_->data.mUPapost), fix16_one );
		MTX_FillDiagonal( &(kf_->data.mDPapost), fix16_from_int(KF_DFLT_ALPHA) );
		MTX_Fill( &(kf_->data.vXapost), 0 );
		if( TRUE == kf_->cfg.bModCntrFlag )
		{
			for(i = 0u; i < kf_->data.vXapost.rows; i++)
			{
				kf_->data.aModCntr[i] = 0;
			}
		}
	}
}

/* same as fa16_dot but without overflow detection and returns a Q48.16 variable */
int64_t KF_48d16Dot(const fix16_t *a, uint_fast8_t a_stride,
                 const fix16_t *b, uint_fast8_t b_stride,
                 uint_fast8_t n)
{
    int64_t sum = 0;

    while (n--)
    {
        if (*a != 0 && *b != 0)
        {
            sum += (int64_t)(*a) * (*b);
        }

        // Go to next item
        a += a_stride;
        b += b_stride;
    }
    if (sum < 0)
    {
        #ifndef FIXMATH_NO_ROUNDING
        // This adjustment is required in order to round -1/2 correctly
        sum--;
        #endif
    }
    sum = (sum >> 16);
    #ifndef FIXMATH_NO_ROUNDING
    sum += (sum & 0x8000) >> 15;
    #endif
    return sum;
}

static StdRtn_t KF_UpdateModuloCounter(KF_Data_t *data_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;
	if(NULL != data_)
	{
		retVal = ERR_OK;
		for(i = 0u; i < data_->vXapost.rows; i++)
		{
			if( (int32_t)(KF_DFLT_MAX_MOD_VAL<<16) <= data_->vXapost.data[i][0] )
			{
				data_->vXapost.data[i][0] %= KF_DFLT_MAX_MOD_VAL;
				data_->aModCntr[i]++;
			}
			else if( -(int32_t)(KF_DFLT_MAX_MOD_VAL<<16) >= data_->vXapost.data[i][0] )
			{
				data_->vXapost.data[i][0] %= KF_DFLT_MAX_MOD_VAL;
				data_->aModCntr[i]--;
			}
		}
	}
	return retVal;
}

static StdRtn_t KF_Predict_x(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t l = 0;
	MTX_t vU = {kf_->cfg.mtx.mGamma.columns, 1, 0, {0u}};
	if(NULL != kf_)
	{
		retVal = ERR_OK;
		MTX_Mult(&(kf_->data.vXapri), &(kf_->cfg.mtx.mPhi), &(kf_->data.vXapost));
		if( (NULL != kf_->cfg.aInptValFct) && (0 != kf_->cfg.mtx.mGamma.columns) )
		{
			for(l = 0u; l < kf_->cfg.mtx.mGamma.columns; l++)
			{
				kf_->cfg.aInptValFct[l](&vU.data[l][0]);
				vU.data[l][0] = fix16_from_int(vU.data[l][0]);
			}
			MTX_Mult(&(kf_->data.vXapost), &(kf_->cfg.mtx.mGamma), &(vU));
			MTX_Add(&(kf_->data.vXapri), &(kf_->data.vXapri), &(kf_->data.vXapost));
		}
	}
	return retVal;
}

static StdRtn_t KF_Predict_P(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	MTX_t mG = kf_->cfg.mtx.mG; /* necessary because G is overwritten in thornton update */
	if( NULL != kf_ )
	{
		retVal       = ERR_OK;
		retVal |= KF_ThorntonTemporalUpdate(&(kf_->data.mUPapri),  &(kf_->data.mDPapri),  &(kf_->cfg.mtx.mPhi),
											&(kf_->data.mUPapost), &(kf_->data.mDPapost), &(mG), &(kf_->cfg.mtx.mQ));
	}
	return retVal;
}

static StdRtn_t KF_Correct(KF_Itm_t *kf_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t m = 0u;
	int32_t dy = 0, ym = 0;
	int64_t ymHat = 0;
	if( (NULL != kf_) && (NULL != kf_->cfg.aMeasValFct) )
	{
		retVal             = ERR_OK;
		kf_->data.vXapost  = kf_->data.vXapri;
		kf_->data.mUPapost = kf_->data.mUPapri;
		kf_->data.mDPapost = kf_->data.mDPapri;
		for(m = 0; m < kf_->cfg.mtx.mH.rows; m++)
		{
			kf_->cfg.aMeasValFct[m](&ym);
			if(TRUE == kf_->cfg.bModCntrFlag)
			{
				ymHat  = KF_48d16Dot( &(kf_->cfg.mtx.mH.data[m][0]), 1, &(kf_->data.vXapost.data[0][0]), FIXMATRIX_MAX_SIZE, kf_->data.vXapost.rows);
				ymHat += (int64_t)( KF_48d16Dot(&(kf_->cfg.mtx.mH.data[m][0]), 1, kf_->data.aModCntr, 1, kf_->data.vXapost.rows) * (KF_DFLT_MAX_MOD_VAL<<16) );
				dy = (int32_t)( (((int64_t)ym)<<16) - ymHat );
			}
			else
			{
				ym <<= 16;
				dy = fix16_sub(ym, fa16_dot( &(kf_->cfg.mtx.mH.data[m][0]), 1, &(kf_->data.vXapost.data[0][0]), FIXMATRIX_MAX_SIZE, kf_->data.vXapost.rows) );
			}
			retVal |= KF_BiermanObservationalUpdate(&(kf_->data.vXapost), &(kf_->data.mUPapost), &(kf_->data.mDPapost),
													dy, kf_->cfg.mtx.mR.data[m][m], &(kf_->cfg.mtx.mH), m);
		}
	}
}

/* TODO overflow handling, reduce number of temp variables with and-logic */
static StdRtn_t KF_ThorntonTemporalUpdate(MTX_t *mUPapri_, MTX_t *mDPapri_, const MTX_t *Phi_, const MTX_t *mUPapost_, const MTX_t *mDPapost_, MTX_t *mGUQ_, const MTX_t *mDQ_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int8_t  i = 0;
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

/* TODO overflow handling, reduce number of temp variables with and-logic */
static StdRtn_t KF_BiermanObservationalUpdate(MTX_t *vXapost_, MTX_t *mUPapost_, MTX_t *mDPapost_, int32_t dym_, int32_t rmm_, const MTX_t *mH_, uint8_t m_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u, j = 0u;
	int32_t alpha = 0, beta = 0, gamma = 0, gammaOld = 0, tmp = 0;
	bool overFlowFlag = FALSE;
	int32_t a[vXapost_->rows], b[vXapost_->rows];

	if( (NULL != vXapost_) && (NULL != mUPapost_) && (NULL != mDPapost_) )
	{
		retVal = ERR_OK;
		/* a = U'h_m', b = Da can be in this loop because D is a diagonal matrix */
		for(i = 0u; i < mUPapost_->rows; i++)
		{
			a[i] = fa16_dot(&(mUPapost_->data[0][i]), FIXMATRIX_MAX_SIZE, &(mH_->data[m_][0]), 1, mUPapost_->rows);
			b[i] = fix16_mul(mDPapost_->data[i][i], a[i]);
		}
		alpha = rmm_;
		gamma = alpha;
		for(j = 0u; j < vXapost_->rows; j++)
		{
			beta     = alpha;
			alpha    = fix16_add( alpha, fix16_mul(a[j], b[j]) );
			gammaOld = gamma;
			gamma    = alpha;
			tmp = fix16_div(mDPapost_->data[j][j], gamma);
			mDPapost_->data[j][j] = fix16_mul(tmp, beta);
			for(i = 0u; i < j; i++)
			{
				beta = mUPapost_->data[i][j];
				tmp = fix16_mul(b[i],a[j]);
				tmp = fix16_div(tmp, gammaOld);
				mUPapost_->data[i][j] = fix16_sub( beta, tmp );
				tmp = fix16_mul(b[j], beta);
				b[i] = fix16_add( b[i], tmp );
			}
		}
		for(i = 0; i < vXapost_->rows; i++) /* update x_apost */
		{
			if ( (fix16_abs(dym_) >= fix16_one) || (fix16_abs(b[i]) >= fix16_one) )
			{
				tmp = fix16_mul(dym_, b[i]);
				if(fix16_overflow == tmp)
				{
					overFlowFlag = TRUE;
				}
				else
				{
					tmp = fix16_div(tmp, gamma);
				}
			}
			if( (fix16_abs(dym_) > fix16_abs(b[i])) && (TRUE == overFlowFlag) )
			{
				tmp = fix16_div(dym_, gamma);
				tmp = fix16_mul(tmp, b[i]);
				overFlowFlag = FALSE;
			}
			if(TRUE == overFlowFlag)
			{
				tmp = fix16_div(b[i], gamma);
				tmp = fix16_mul(tmp, dym_);
				overFlowFlag = FALSE;
			}
			vXapost_->data[i][0] = fix16_add(vXapost_->data[i][0], tmp);
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
			if( TRUE == KF_pTbl->aKfs[i].cfg.bModCntrFlag )
			{
				KF_UpdateModuloCounter( &(KF_pTbl->aKfs[i].data) );
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
			retVal = ERR_OK;
			if(TRUE == KF_pTbl->aKfs[idx_].cfg.bModCntrFlag)
			{
				*pVal_ = KF_pTbl->aKfs[idx_].data.aModCntr[1]*KF_DFLT_MAX_MOD_VAL + (KF_pTbl->aKfs[idx_].data.vXapost.data[1][0]>>16);
			}
			else
			{
				*pVal_ = (int16_t)((KF_pTbl->aKfs[idx_].data.vXapost.data[1][0])>>16);
			}
		}
	}
	return retVal;
}


#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
