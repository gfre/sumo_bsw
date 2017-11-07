/*************************************************************************************************
 * @file		kf.c
 * @ingroup		kf Kalman FIlter
 * @brief 		This Module implements a Kalman Filter for the velocity given the current position.
 *
 *		This file contains all the logic for the kalman filtering. It contains functions to deal with matrix operations,
 *		matrix vector operations, and vector operations such as multiplying, transposing, etc. The init function
 *		transposes the A (system) matrix and c (measurement) vector and stores it in a global variable. It also
 *		receives the configuration from kf_cfg.c.
 *		The main function implements a recursive kalman filter algorithm and is currently called as part of the "drive" module.
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
		MTX_Reset( &(kf_->data.mPrvErrCoVar) );
		MTX_Reset( &(kf_->data.vPrvStEst) );
		kf_->data.nMdCntr = 0;
		for(i = 0u; i < kf_->cfg.dim->nMsrdSts; i++)
		{
			kf_->cfg.aMeasValFct[i]( &(MTXLOC_ij( (kf_->data.vPrvStEst), i, 0) ) );
			MTXLOC_ij(kf_->data.vPrvStEst, i, 0) = KF_UPSCALE( MTXLOC_ij(kf_->data.vPrvStEst, i, 0), kf_->cfg.scl->nStVec );
		}
	}
	else
	{
		/* error handling */
	}
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void KF_Init(void)
{
	uint8_t i = 0u;
	KF_pTbl = Get_pKfItmTbl();
	if( (NULL != KF_pTbl) && (NULL != KF_pTbl->aTls) )
	{
		for(i = 0u; i < KF_pTbl->numTls; i++)
		{
			KF_Reset(&KF_pTbl->aTls[i]);
		}
	}
	else
	{
		/* error handling */
	}
}

StdRtn_t KF_Read_i16EstdVal(int16_t *pVal_, const uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( (NULL != pVal_) && (NULL != KF_pTbl) && (NULL != KF_pTbl->aTls) )
	{
		retVal = ERR_PARAM_VALUE;
		if(idx_ < KF_pTbl->numTls )
		{
			*pVal_ = MTXLOC_ij( KF_pTbl->aTls[idx_].data.vPrvStEst, idx_, 0 );
			retVal = ERR_OK;
		}
	}
	return retVal;
}



#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
