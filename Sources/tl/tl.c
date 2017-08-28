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
#include "tl_api.h"
#include "pid_api.h"
#include "tacho_api.h"



/*======================================= >> #DEFINES << =========================================*/
#define TL_CONDITIONAL_RETURN(condVar_, trueVal_, falseVal_) ( (TRUE == condVar_)?(trueVal_) : (falseVal_) )
#define TL_CalcFilteredVal( plant_,  result_ )    ( PID( plant_,  result_ ) )

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static PID_Itm_t* TL_LftItm  = NULL;
static PID_Itm_t* TL_RghtItm = NULL;
static int32_t TL_CurLftSpd = 0, TL_CurRghtSpd = 0;
static int32_t TL_CurLftPos = 0, TL_CurRghtPos = 0; //scaled to 1000


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
int32_t TL_Get_Speed(bool isLeft_)
{
	return TL_CONDITIONAL_RETURN(isLeft_, TL_CurLftSpd, TL_CurRghtSpd);
}

StdRtn_t TL_Read_CurLftPos(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != result_)
	{
		*result_ = TL_CurLftPos/1000;
		retVal 	= ERR_OK;
	}
	return retVal;
}

StdRtn_t TL_Read_CurRghtPos(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != result_)
	{
		*result_ = TL_CurRghtPos/1000;
		retVal 	= ERR_OK;
	}
	return retVal;
}

void TL_RunStartup()
{
	uint8_t i = 0u;
	PID_Cfg_t* tlCfg = NULL;
	NVM_PidCfg_t pidPrm = {0u};
	StdRtn_t retVal = ERR_VALUE;

	tlCfg = Get_pTLCfg();
	if(NULL != tlCfg)
	{
		for(i = 0; i < tlCfg->NumOfItms; i++)
		{
			if(NULL != &tlCfg->pItmTbl[i])
			{
				if(NULL != tlCfg->pItmTbl[i].pNVMReadValFct)
				{
					retVal = tlCfg->pItmTbl[i].pNVMReadValFct(&pidPrm);
					tlCfg->pItmTbl[i].Config->Factor_KP_scld = (uint32_t)pidPrm.KP_scld;
					tlCfg->pItmTbl[i].Config->Factor_KI_scld = (uint32_t)pidPrm.KI_scld;
					tlCfg->pItmTbl[i].Config->Factor_KD_scld = (uint32_t)pidPrm.KD_scld;
					tlCfg->pItmTbl[i].Config->Scale		     = (uint16_t)pidPrm.Scale;
					tlCfg->pItmTbl[i].Config->SaturationVal  = (uint32_t)pidPrm.SaturationVal;
				}
				if ( (NULL != tlCfg->pItmTbl[i].pNVMReadDfltValFct) && (ERR_OK != retVal) )
				{
					tlCfg->pItmTbl[i].pNVMReadDfltValFct(&pidPrm);
					tlCfg->pItmTbl[i].Config->Factor_KP_scld = (uint32_t)pidPrm.KP_scld;
					tlCfg->pItmTbl[i].Config->Factor_KI_scld = (uint32_t)pidPrm.KI_scld;
					tlCfg->pItmTbl[i].Config->Factor_KD_scld = (uint32_t)pidPrm.KD_scld;
					tlCfg->pItmTbl[i].Config->Scale		     = (uint16_t)pidPrm.Scale;
					tlCfg->pItmTbl[i].Config->SaturationVal  = (uint32_t)pidPrm.SaturationVal;
				}
				else
				{
					/* take initialized values from pid_cfg.c */
				}
				tlCfg->pItmTbl[i].Saturation  = PID_NO_SAT;
				tlCfg->pItmTbl[i].integralVal = 0;
				tlCfg->pItmTbl[i].lastError   = 0;
			}
		}
	}
	else
	{
		/* error handling */
	}
	TL_LftItm  = &tlCfg->pItmTbl[TL_LFT_SPD_EST];
	TL_RghtItm = &tlCfg->pItmTbl[TL_RGHT_SPD_EST];
}

void TL_Reset()
{
	TL_LftItm->integralVal = 0;
	TL_LftItm->Saturation  = PID_NO_SAT;
	TL_LftItm->lastError   = 0;

	TL_RghtItm->integralVal = 0;
	TL_RghtItm->Saturation  = PID_NO_SAT;
	TL_RghtItm->lastError   = 0;

	TL_LftItm->pCurValFct(&TL_CurLftPos);
	TL_RghtItm->pCurValFct(&TL_CurRghtPos);
	TL_CurLftPos  *= 1000;
	TL_CurRghtPos *= 1000;
	TL_CurLftSpd  = 0;
	TL_CurRghtSpd = 0;
}

void TL_CalcSpd()
{
	StdRtn_t retVal = ERR_OK;
	retVal |= TL_CalcFilteredVal(TL_LftItm,  &TL_CurLftSpd);
	retVal |= TL_CalcFilteredVal(TL_RghtItm, &TL_CurRghtSpd);
	TL_CurLftPos  += TL_CurLftSpd * TACHO_SAMPLE_PERIOD_MS;
	TL_CurRghtPos += TL_CurRghtSpd * TACHO_SAMPLE_PERIOD_MS;

	if (ERR_OK != retVal)
	{
		/* error handling */
	}
}

#ifdef MASTER_tl_C_
#undef MASTER_tl_C_
#endif /* !MASTER_tl_C_ */
