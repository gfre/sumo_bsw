/***********************************************************************************************//**
 * @file		tl.c
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	S. Helling stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.08.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_tl_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tl.h"
#include "tl_cfg.h"
#include "tl_api.h"

#include "pid_api.h"




/*======================================= >> #DEFINES << =========================================*/
#define TL_CONDITIONAL_RETURN(condVar_, trueVal_, falseVal_) ( (TRUE == condVar_)?(trueVal_) : (falseVal_) )
#define TL_CALC_FILTERED_SIGNAL( plant_,  result_ )    ( PID( plant_,  result_ ) )

#define TL_DOWNSACLE(val_) ( (val_)/(1000) )
#define TL_UPSACLE(val_) ( (val_)*(1000) )

/*TODO */
#define TL_SAMPLE_TIME (5);

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static inline void TL_Reset(TL_Itm_t *tl_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/
TL_ItmTbl_t* pTbl = NULL;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static inline void TL_Reset(TL_Itm_t *tl_)
{
	if( NULL != tl_ )
	{
		tl_->cfg.Saturation  = PID_NO_SAT;
		tl_->cfg.integralVal = 0;
		tl_->cfg.lastError   = 0;
		tl_->data.dfltrdValdt= 0;
		if( NULL != tl_->cfg.pCurValFct )
		{
			tl_->cfg.pCurValFct(&tl_->data.fltrdVal);
			tl_->data.fltrdVal  = TL_UPSACLE(tl_->data.fltrdVal);
		}
	}
	else
	{
		/* error handling */
	}
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/


void TL_Init(void)
{
	uint8_t i = 0u;
	NVM_PidCfg_t pidPrm = {0u};
	StdRtn_t errVal = ERR_VALUE;

	pTbl = Get_pTlItmTbl();
	if( (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		for(i = 0u; i < pTbl->numTls; i++)
		{
			if(NULL != pTbl->aTls[i].cfg.pNVMReadValFct)
			{
				if( ERR_OK == pTbl->aTls[i].cfg.pNVMReadValFct(&pidPrm) )
				{
					errVal = ERR_OK;
					pTbl->aTls[i].cfg.Config->Factor_KP_scld = (uint32_t)pidPrm.KP_scld;
					pTbl->aTls[i].cfg.Config->Factor_KI_scld = (uint32_t)pidPrm.KI_scld;
					pTbl->aTls[i].cfg.Config->Scale		     = (uint16_t)pidPrm.Scale;
					pTbl->aTls[i].cfg.Config->SaturationVal  = (uint32_t)pidPrm.SaturationVal;
				}
			}

			if ( (ERR_OK != errVal) && (NULL != pTbl->aTls[i].cfg.pNVMReadDfltValFct) )
			{
				errVal = ERR_VALUE;
				if( ERR_OK == pTbl->aTls[i].cfg.pNVMReadDfltValFct(&pidPrm) )
				{
					errVal = ERR_OK;
					pTbl->aTls[i].cfg.Config->Factor_KP_scld = (uint32_t)pidPrm.KP_scld;
					pTbl->aTls[i].cfg.Config->Factor_KI_scld = (uint32_t)pidPrm.KI_scld;
					pTbl->aTls[i].cfg.Config->Scale		     = (uint16_t)pidPrm.Scale;
					pTbl->aTls[i].cfg.Config->SaturationVal  = (uint32_t)pidPrm.SaturationVal;
				}
			}
			/* PI-control only, D-part is always 0 for a tracking loop */
			pTbl->aTls[i].cfg.Config->Factor_KD_scld = 0u;

			if (ERR_OK != errVal)
			{
				/* error handling */
			}

			TL_Reset(&pTbl->aTls[i]);
		}
	}
	else
	{
		/* error handling */
	}


}

void TL_Main(void)
{
	StdRtn_t retVal = ERR_OK;
	uint8_t i = 0u;
	if( (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		for(i = 0u; i < pTbl->numTls; i++)
		{
			/* Get the filtered derivative of the signal value */
			if( ERR_OK == TL_CALC_FILTERED_SIGNAL(&pTbl->aTls[i].cfg,  &pTbl->aTls[i].data.dfltrdValdt) )
			{
				/* Euler forward integration of the filtered derivative to get the filtered signal value*/
				pTbl->aTls[i].data.fltrdVal  += pTbl->aTls[i].data.dfltrdValdt * TL_SAMPLE_TIME;
			}
			else
			{
				/* error handling */
			}
		}
	}
	else
	{
		/* error handling */
	}
}

void TL_DeInit(void)
{
	pTbl = NULL;
}


StdRtn_t TL_Read_i32FltrdVal(int32_t* pSig_, const uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( ( NULL != pSig_ ) && (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		retVal = ERR_PARAM_VALUE;
		if( idx_< pTbl->numTls )
		{
			*pSig_ = TL_DOWNSACLE(pTbl->aTls[idx_].data.fltrdVal);
			retVal 	= ERR_OK;
		}
	}
	return retVal;
}

StdRtn_t TL_Read_i32dFltrdValdt(int32_t* pSig_, const uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( ( NULL != pSig_ ) && (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		retVal = ERR_PARAM_VALUE;
		if( idx_< pTbl->numTls )
		{
			*pSig_ = pTbl->aTls[idx_].data.dfltrdValdt;
			retVal 	= ERR_OK;
		}
	}
	return retVal;
}

StdRtn_t TL_Read_vFltrdVal(void* pSig_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if( ( NULL != pSig_ ) && (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		retVal = ERR_PARAM_VALUE;
		TL_vReadVal_t *pSig = (TL_vReadVal_t *)pSig_;
		if( pSig->idx < pTbl->numTls )
		{
			pSig->val = pTbl->aTls[pSig->idx].data.fltrdVal;
			retVal 	= ERR_OK;
		}
	}
	return retVal;
}



StdRtn_t TL_Read_vdFltrdValdt(void* pSig_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if( ( NULL != pSig_ ) && (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		retVal = ERR_PARAM_VALUE;
		TL_vReadVal_t *pSig = (TL_vReadVal_t *)pSig_;
		if( pSig->idx < pTbl->numTls )
		{
			pSig->val = TL_DOWNSACLE(pTbl->aTls[pSig->idx].data.dfltrdValdt);
			retVal 	= ERR_OK;
		}
	}
	return retVal;
}


#ifdef MASTER_tl_C_
#undef MASTER_tl_C_
#endif /* !MASTER_tl_C_ */
