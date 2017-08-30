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
#include "pid_cfg.h"
#include "pid_api.h"
#include "mot_api.h"
#include "nvm_api.h"



/*======================================= >> #DEFINES << =========================================*/
#define PID_DEBUG 0 /* careful: this will slow down the PID loop frequency! */

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static PID_ItmTbl_t *pPidTbl = NULL;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t PIDext(int32_t setVal_, int32_t actVal_, const PID_Gain_t *gain_, PID_Data_t *data_, int32_t* ctrlVal_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t err = 0, pTerm = 0, dTerm = 0, u = 0;

	if( ( NULL != ctrlVal_ ) && ( NULL != data_) )
	{
		retVal = ERR_OK;

		/* Calculate current error value */
		err = setVal_ - actVal_;

		/* Integration part and anti windup part */
		if ( (err > -10) && (err < 10) )  /* avoid jitter around zero */
		{
			err = 0;
		}

		if( ( (data_->sat <= PID_NEG_SAT)  && (err < 0) ) || ( (data_->sat >= PID_POS_SAT) && (err > 0) ) )
		{
			/* don't allow integrating in direction of saturation -> do nothing */
		}
		else //allow integrating in opposite direction to saturation
		{
			data_->intVal += ( ( ((int32_t)gain_->kI_scld)*err ) / ( (int32_t)gain_->nScale ) );
		}

		if( data_->intVal <= -((int32_t)gain_->intSatVal))
		{
			data_->intVal = -((int32_t)gain_->intSatVal);
			data_->sat  = PID_NEG_SAT;
		}
		else if(data_->intVal >= ((int32_t)gain_->intSatVal) )
		{
			data_->intVal = ((int32_t)gain_->intSatVal);
			data_->sat  = PID_POS_SAT;
		}
		else
		{
			data_->sat = PID_NO_SAT;
		}

		/* Proportional part */
		pTerm = ( ((int32_t)gain_->kP_scld)*err ) / ( (int32_t)gain_->nScale );

		if( pTerm < -((int32_t)gain_->intSatVal) )
		{
			pTerm = -((int32_t)gain_->intSatVal);
		}
		else if( pTerm > (int32_t)(gain_->intSatVal) )
		{
			pTerm =  (int32_t)gain_->intSatVal;
		}

		/* Derivative part */
		dTerm = ( (err - data_->prevErr) * (int32_t)gain_->kD_scld ) / ( (int32_t)gain_->nScale );
		data_->prevErr = err;


		/* Calculate and bound output */
		u = pTerm + dTerm + data_->intVal;
		if(u < -((int32_t)gain_->intSatVal))     u = -((int32_t)gain_->intSatVal);
		else if(u > ((int32_t)gain_->intSatVal)) u =  ((int32_t)gain_->intSatVal);

		*ctrlVal_ = u;
	}
	return retVal;
}

StdRtn_t PID(int32_t setVal_, int32_t actVal_, uint8_t idx_, int32_t* ctrlVal_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if( ( NULL != ctrlVal_ ) && ( NULL != pPidTbl ) && ( NULL != pPidTbl->aPids )  )
	{
		retVal = ERR_PARAM_INDEX;

		if(idx_ < pPidTbl->numPids)
		{
			retVal = PIDext(setVal_, actVal_, &pPidTbl->aPids[idx_].cfg.gain, &pPidTbl->aPids[idx_].data, ctrlVal_);
		}
	}
	return retVal;
}

StdRtn_t PID_Reset(uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if( ( NULL != pPidTbl ) && ( NULL != pPidTbl->aPids )  )
	{
		retVal = ERR_PARAM_INDEX;

		if(idx_ < pPidTbl->numPids)
		{
			pPidTbl->aPids[idx_].data.intVal = 0;
			pPidTbl->aPids[idx_].data.prevErr = 0;
			pPidTbl->aPids[idx_].data.sat = PID_NO_SAT;
		}
	}
}

void PID_Init(void)
{
	uint8_t i = 0u;
	NVM_PidCfg_t nvmPid = {0};
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	pPidTbl = Get_pPidItmTbl();

	if( NULL != pPidTbl )
	{
		for(i = 0u; i < pPidTbl->numPids; i++)
		{
			if(NULL != &pPidTbl->aPids[i])
			{
				if(NULL != pPidTbl->aPids[i].cfg.nvm.readFct)
				{
					retVal = ERR_VALUE;
					retVal = pPidTbl->aPids[i].cfg.nvm.readFct(&nvmPid);
					pPidTbl->aPids[i].cfg.gain.kP_scld = nvmPid.KP_scld;
					pPidTbl->aPids[i].cfg.gain.kI_scld = nvmPid.KI_scld;
					pPidTbl->aPids[i].cfg.gain.kD_scld = nvmPid.KD_scld;
					pPidTbl->aPids[i].cfg.gain.nScale  = nvmPid.Scale;
					pPidTbl->aPids[i].cfg.gain.intSatVal  = nvmPid.SaturationVal;
				}
				if ( (NULL != pPidTbl->aPids[i].cfg.nvm.readDfltFct) && (ERR_OK != retVal) )
				{
					retVal = pPidTbl->aPids[i].cfg.nvm.readFct(&nvmPid);
					pPidTbl->aPids[i].cfg.gain.kP_scld = nvmPid.KP_scld;
					pPidTbl->aPids[i].cfg.gain.kI_scld = nvmPid.KI_scld;
					pPidTbl->aPids[i].cfg.gain.kD_scld = nvmPid.KD_scld;
					pPidTbl->aPids[i].cfg.gain.nScale  = nvmPid.Scale;
					pPidTbl->aPids[i].cfg.gain.intSatVal  = nvmPid.SaturationVal;
				}
				else
				{
					/* take initialized values from pid_cfg.c */
				}
				pPidTbl->aPids[i].data.sat = PID_NO_SAT;
				pPidTbl->aPids[i].data.intVal  = 0;
				pPidTbl->aPids[i].data.prevErr = 0;
			}
		}
	}
	else
	{
		/* error handling */
	}
}

void PID_DeInit(void)
{
	pPidTbl = NULL;
}

#ifdef MASTER_pid_C_
#undef MASTER_pid_C_
#endif /* !MASTER_pid_C_ */

