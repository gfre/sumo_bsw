/***********************************************************************************************//**
 * @file		pid.c
 * @ingroup		pid
 * @brief 		Implementation of PID controllers.
 *
 * This module implements PID controllers for position and speed control of the Sumo robots. An Anti-
 * Wind-Up algorithm avoids drifting of the integral part and the maximum allowed control value can
 * changed by parameter. Controller parameters are read from the [NVM software component](@ref nvm)
 * during initialisation and may be changed via [command line shell](@ref sh).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.01.2017
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/


#define MASTER_pid_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "pid.h"
#include "pid_api.h"
#include "mot_api.h"
#include "nvm_api.h"



/*======================================= >> #DEFINES << =========================================*/
#define PID_DEBUG 0 /* careful: this will slow down the PID loop frequency! */

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t PID(PID_Itm_t* itm_, int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t pTerm = 0, dTerm = 0, u = 0, error = 0, trgt = 0, cur = 0;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		retVal |= itm_->pTrgtValFct(&trgt);
		retVal |= itm_->pCurValFct(&cur);
		error = trgt - cur;

	/* Integration and anti windup part */
		if( (PID_LFT_MTR_POS == itm_->ItmType) || (PID_RGHT_MTR_POS == itm_->ItmType) ) // TODO could be part of itm_: "bool avoidJitter" and "int32_t JitterVal"?
		{
			if ( (error > -10) && (error < 10) )  /* avoid jitter around zero */
			{
				error = 0;
			}
		}

		if( ( (itm_->Saturation <= PID_NEG_SAT)  && (error < 0) ) || ( (itm_->Saturation >= PID_POS_SAT) && (error > 0) ) )
		{
			/* don't allow integrating in direction of saturation -> do nothing */
		}
		else //allow integrating in opposite direction to saturation
		{
			itm_->integralVal += ( (((int32_t)itm_->Config->Factor_KI_scld)*error)/((int32_t)itm_->Config->Scale) );
		}

		if( (itm_->integralVal > -((int32_t)itm_->Config->SaturationVal)) && (itm_->integralVal < (int32_t)itm_->Config->SaturationVal) )
		{
			itm_->Saturation = PID_NO_SAT;
		}
		else if(itm_->integralVal < -((int32_t)itm_->Config->SaturationVal))
		{
			itm_->integralVal = -((int32_t)itm_->Config->SaturationVal);
			itm_->Saturation  = PID_NEG_SAT;
		}
		else if(itm_->integralVal > ((int32_t)itm_->Config->SaturationVal) )
		{
			itm_->integralVal = ((int32_t)itm_->Config->SaturationVal);
			itm_->Saturation  = PID_POS_SAT;
		}

	/* Proportional part */
		pTerm = ( (((int32_t)itm_->Config->Factor_KP_scld)*error)/((int32_t)itm_->Config->Scale) );

		if(pTerm < -((int32_t)itm_->Config->SaturationVal))     pTerm = -((int32_t)itm_->Config->SaturationVal);
		else if(pTerm > (int32_t)(itm_->Config->SaturationVal)) pTerm =  ((int32_t)itm_->Config->SaturationVal);

	/* Derivative part */
		dTerm = ( (error - (int32_t)itm_->lastError) * (int32_t)itm_->Config->Factor_KD_scld ) / ( (int32_t)itm_->Config->Scale );
		itm_->lastError = error;

	/* Calculate and bound output */
		u = pTerm + dTerm + itm_->integralVal;
		if(u < -((int32_t)itm_->Config->SaturationVal))     u = -((int32_t)itm_->Config->SaturationVal);
		else if(u > ((int32_t)itm_->Config->SaturationVal)) u =  ((int32_t)itm_->Config->SaturationVal);

		*result_ = u;
	}
	return retVal;
}


void PID_Init(void)
{
	uint8_t i = 0u;
	PID_Cfg_t* pidCfg = NULL;
	NVM_PidCfg_t pidPrm = {0u};
	StdRtn_t retVal = ERR_VALUE;

	pidCfg = Get_pPidCfg();
	if(NULL != pidCfg)
	{
		for(i = 0; i < pidCfg->NumOfItms; i++)
		{
			if(NULL != &pidCfg->pItmTbl[i])
			{
				if(NULL != pidCfg->pItmTbl[i].pNVMReadValFct)
				{
					retVal = pidCfg->pItmTbl[i].pNVMReadValFct(&pidPrm);
					pidCfg->pItmTbl[i].Config->Factor_KP_scld = (uint32_t)pidPrm.KP_scld;
					pidCfg->pItmTbl[i].Config->Factor_KI_scld = (uint32_t)pidPrm.KI_scld;
					pidCfg->pItmTbl[i].Config->Factor_KD_scld = (uint32_t)pidPrm.KD_scld;
					pidCfg->pItmTbl[i].Config->Scale		  = (uint16_t)pidPrm.Scale;
					pidCfg->pItmTbl[i].Config->SaturationVal  = (uint32_t)pidPrm.SaturationVal;
				}
				if ( (NULL != pidCfg->pItmTbl[i].pNVMReadDfltValFct) && (ERR_OK != retVal) )
				{
					pidCfg->pItmTbl[i].pNVMReadDfltValFct(&pidPrm);
					pidCfg->pItmTbl[i].Config->Factor_KP_scld = (uint32_t)pidPrm.KP_scld;
					pidCfg->pItmTbl[i].Config->Factor_KI_scld = (uint32_t)pidPrm.KI_scld;
					pidCfg->pItmTbl[i].Config->Factor_KD_scld = (uint32_t)pidPrm.KD_scld;
					pidCfg->pItmTbl[i].Config->Scale		  = (uint16_t)pidPrm.Scale;
					pidCfg->pItmTbl[i].Config->SaturationVal  = (uint32_t)pidPrm.SaturationVal;
				}
				else
				{
					/* take initialized values from pid_cfg.c */
				}
				pidCfg->pItmTbl[i].Saturation  = PID_NO_SAT;
				pidCfg->pItmTbl[i].integralVal = 0;
				pidCfg->pItmTbl[i].lastError   = 0;
			}
		}
	}
	else
	{
		/* error handling */
	}
}



#ifdef MASTER_pid_C_
#undef MASTER_pid_C_
#endif /* !MASTER_pid_C_ */

