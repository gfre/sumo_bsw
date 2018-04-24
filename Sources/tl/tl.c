/***********************************************************************************************//**
 * @file		tl.c
 * @ingroup		tl
 * @brief 		Implementation of a tracking loop filter algorithm
 *
 * This component implements the core estimation algorithm for the Tracking Loop Filter.
 * In order to do this, it makes use of the component @ref pid.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author 	S. Helling        stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.08.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_tl_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tl.h"
#include "tl_cfg.h"
#include "tl_api.h"

/* Application APIs */
#include "pid_api.h"



/*======================================= >> #DEFINES << =========================================*/
/*
 * Macro for abstraction of the function which calculates the filtered signal
 */
#define TL_CALC_FILTERED_SIGNAL( measVal_, fltrdVal_, pPidGain_, pPidData_, pCtrlVal_ )  \
	( PIDext( measVal_,  fltrdVal_, pPidGain_, pPidData_, pCtrlVal_) )
/*
 * Macro to scale a value down
 */
#define TL_DOWNSACLE(val_) ( (val_)/(1000) )

/*
 * Macro to scale a value up
 */
#define TL_UPSACLE(val_) ( (val_)*(1000) )



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void TL_Reset(TL_Itm_t *tl_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
TL_ItmTbl_t* pTbl = NULL;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void TL_Reset(TL_Itm_t *tl_)
{
	if( NULL != tl_ )
	{
		tl_->data.pid.sat  = PID_NO_SAT;
		tl_->data.pid.intVal = 0;
		tl_->data.pid.prevErr = 0;
		tl_->data.dfltrdValdt= 0;
		if( NULL != tl_->cfg.measValFct )
		{
			tl_->cfg.measValFct(&tl_->data.fltrdVal);
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
	StdRtn_t errVal = ERR_PARAM_ADDRESS;

	pTbl = Get_pTlItmTbl();
	if( (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		errVal = ERR_OK;
		for(i = 0u; i < pTbl->numTls; i++)
		{
#if TL_USES_NVM
			errVal = ERR_VALUE;
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
			if (ERR_OK != errVal)
			{
				/* error handling */
			}
#endif
			/* PI-control only, D-part is always 0 for a tracking loop */
			pTbl->aTls[i].cfg.pid.kD_scld = 0u;

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
	int32_t measVal = 0;
	int32_t fltrdVal = 0;
	int32_t dfltrdVal = 0;

	if( (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		for(i = 0u; i < pTbl->numTls; i++)
		{
			/* Down-scale internal filtered value */
			fltrdVal = TL_DOWNSACLE(pTbl->aTls[i].data.fltrdVal);
			/* Read measured value */
			retVal |= pTbl->aTls[i].cfg.measValFct(&measVal);
			/* Get the filtered derivative of the signal value */
			retVal |= TL_CALC_FILTERED_SIGNAL(measVal, fltrdVal, &pTbl->aTls[i].cfg.pid,
					&pTbl->aTls[i].data.pid, &dfltrdVal);
			if( ERR_OK == retVal )
			{
				pTbl->aTls[i].data.dfltrdValdt = (int16_t)dfltrdVal;

				/* Euler backward integration of the filtered derivative to get the filtered signal value*/
				/* Filtered values is up-scaled due integer multiplication with sample time in MS. */
				pTbl->aTls[i].data.fltrdVal  += dfltrdVal * (int32_t)pTbl->aTls[i].cfg.smplTimeMS;
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


StdRtn_t TL_Read_i32FltrdVal(int32_t* pVal_, const uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( ( NULL != pVal_ ) && (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		retVal = ERR_PARAM_INDEX;
		if( idx_< pTbl->numTls )
		{
			*pVal_ = TL_DOWNSACLE(pTbl->aTls[idx_].data.fltrdVal);
			retVal 	= ERR_OK;
		}
	}
	return retVal;
}

StdRtn_t TL_Read_i16dFltrdValdt(int16_t* pVal_, const uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( ( NULL != pVal_ ) && (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		retVal = ERR_PARAM_INDEX;
		if( idx_< pTbl->numTls )
		{
			*pVal_ = pTbl->aTls[idx_].data.dfltrdValdt;
			retVal 	= ERR_OK;
		}
	}
	return retVal;
}

StdRtn_t TL_Read_vFltrdVal(void* pVal_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if( ( NULL != pVal_ ) && (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		retVal = ERR_PARAM_INDEX;
		TL_vReadVal_t *pVal = (TL_vReadVal_t *)pVal_;
		if( pVal->idx < pTbl->numTls )
		{
			pVal->val = TL_DOWNSACLE(pTbl->aTls[pVal->idx].data.fltrdVal);
			retVal 	= ERR_OK;
		}
	}
	return retVal;
}



StdRtn_t TL_Read_vdFltrdValdt(void* pVal_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if( ( NULL != pVal_ ) && (NULL != pTbl) && ( NULL != pTbl->aTls) )
	{
		retVal = ERR_PARAM_INDEX;
		TL_vReadVal_t *pVal = (TL_vReadVal_t *)pVal_;
		if( pVal->idx < pTbl->numTls )
		{
			pVal->val = pTbl->aTls[pVal->idx].data.dfltrdValdt;
			retVal 	= ERR_OK;
		}
	}
	return retVal;
}


#ifdef MASTER_tl_C_
#undef MASTER_tl_C_
#endif /* !MASTER_tl_C_ */
