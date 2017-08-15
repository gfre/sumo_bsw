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
StdRtn_t PI(PID_Plant_t* plant_, int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	int32_t pTerm = 0, dTerm = 0, u = 0, error = 0, trgt = 0, cur = 0;
	if(NULL != result_)
	{
		retVal = ERR_OK;
		retVal |= plant_->pTrgtValFct(&trgt);
		retVal |= plant_->pCurValFct(&cur);
		error = trgt - cur;

		if( (PID_LEFT_MOTOR_POS == plant_->PlantType) || (PID_RIGHT_MOTOR_POS == plant_->PlantType) ) //teil der config? so nicht allgemein
		{
			if ( (error > -10) && (error<10) )  /* avoid jitter around zero */
			{
				error = 0;
			}
		}

		if( ( (plant_->Saturation <= PID_NEG_SAT)  && (error < 0) ) || ( (plant_->Saturation >= PID_POS_SAT) && (error > 0) ) )
		{
			/* don't allow integrating in direction of saturation -> do nothing */
		}
		else //allow integrating in opposite direction to saturation
		{
			plant_->integralVal += ( (((int32_t)plant_->Config->Factor_KI_scld)*error)/((int32_t)plant_->Config->Scale) );
		}

		if( (plant_->integralVal > -((int32_t)plant_->Config->iWindUpMaxVal)) && (plant_->integralVal < (int32_t)plant_->Config->iWindUpMaxVal) )
		{
			plant_->Saturation = PID_NO_SAT;
		}
		else if(plant_->integralVal < -((int32_t)plant_->Config->iWindUpMaxVal))
		{
			plant_->integralVal = -((int32_t)plant_->Config->iWindUpMaxVal);
			plant_->Saturation  = PID_NEG_SAT;
		}
		else if(plant_->integralVal > ((int32_t)plant_->Config->iWindUpMaxVal) )
		{
			plant_->integralVal = ((int32_t)plant_->Config->iWindUpMaxVal);
			plant_->Saturation  = PID_POS_SAT;
		}

		pTerm = ( (((int32_t)plant_->Config->Factor_KP_scld)*error)/((int32_t)plant_->Config->Scale) );

		if(pTerm < -((int32_t)plant_->Config->iWindUpMaxVal))     pTerm = -((int32_t)plant_->Config->iWindUpMaxVal);
		else if(pTerm > (int32_t)(plant_->Config->iWindUpMaxVal)) pTerm =  ((int32_t)plant_->Config->iWindUpMaxVal);

		u = pTerm + plant_->integralVal;
		if(u < -((int32_t)plant_->Config->iWindUpMaxVal))     u = -((int32_t)plant_->Config->iWindUpMaxVal);
		else if(u > ((int32_t)plant_->Config->iWindUpMaxVal)) u =  ((int32_t)plant_->Config->iWindUpMaxVal);

		*result_ = u;
	}
	return retVal;
}


void PID_Init(void)
{
	NVM_PidCfg_t pidCfg = {0u};
	PID_Plant_t* posLeftPlant = NULL;
	PID_Plant_t* posRightPlant = NULL;
	PID_Plant_t* speedLeftPlant = NULL;
	PID_Plant_t* speedRightPlant = NULL;
	posLeftPlant    = &Get_pPidCfg()->pPlantTbl[PID_LEFT_MOTOR_POS];
	posRightPlant   = &Get_pPidCfg()->pPlantTbl[PID_RIGHT_MOTOR_POS];
	speedLeftPlant  = &Get_pPidCfg()->pPlantTbl[PID_LEFT_MOTOR_SPEED];
	speedRightPlant = &Get_pPidCfg()->pPlantTbl[PID_RIGHT_MOTOR_SPEED];

	if ( ERR_OK == NVM_Read_PIDPosCfg(&pidCfg) )
	{
		posLeftPlant->Config->Factor_KP_scld = (int32_t)pidCfg.KP_scld;
		posLeftPlant->Config->Factor_KI_scld = (int32_t)pidCfg.KI_scld;
		posLeftPlant->Config->Factor_KD_scld = (int32_t)pidCfg.KD_scld;
		posLeftPlant->Config->iWindUpMaxVal  = (int32_t)pidCfg.iWindupMaxVal;
		posLeftPlant->Config->Scale          = (int32_t)pidCfg.Scale;
	}
	else if(ERR_OK == NVM_Read_Dflt_PIDPosCfg(&pidCfg))
	{
		posLeftPlant->Config->Factor_KP_scld = (int32_t)pidCfg.KP_scld;
		posLeftPlant->Config->Factor_KI_scld = (int32_t)pidCfg.KI_scld;
		posLeftPlant->Config->Factor_KD_scld = (int32_t)pidCfg.KD_scld;
		posLeftPlant->Config->iWindUpMaxVal  = (int32_t)pidCfg.iWindupMaxVal;
		posLeftPlant->Config->Scale   		= (int32_t)pidCfg.Scale;
		NVM_Save_PIDPosCfg(&pidCfg);
	}
	else
	{
		/* error handling */
	}
	posLeftPlant->lastError   = 0;
	posLeftPlant->integralVal = 0;



	posRightPlant->Config->Factor_KP_scld = posLeftPlant->Config->Factor_KP_scld;
	posRightPlant->Config->Factor_KI_scld = posLeftPlant->Config->Factor_KI_scld;
	posRightPlant->Config->Factor_KD_scld = posLeftPlant->Config->Factor_KD_scld;
	posRightPlant->Config->iWindUpMaxVal  = posLeftPlant->Config->iWindUpMaxVal;
	posRightPlant->Config->Scale   		  = posLeftPlant->Config->Scale;
	posRightPlant->lastError   = posLeftPlant->lastError;
	posRightPlant->integralVal = posLeftPlant->integralVal;


	if ( ERR_OK == NVM_Read_PIDSpdLeCfg(&pidCfg) )
	{
		speedLeftPlant->Config->Factor_KP_scld = (int32_t)pidCfg.KP_scld;
		speedLeftPlant->Config->Factor_KI_scld = (int32_t)pidCfg.KI_scld;
		speedLeftPlant->Config->Factor_KD_scld = (int32_t)pidCfg.KD_scld;
		speedLeftPlant->Config->iWindUpMaxVal  = (int32_t)pidCfg.iWindupMaxVal;
		speedLeftPlant->Config->Scale 		  = (int32_t)pidCfg.Scale;
	}
	else if(ERR_OK == NVM_Read_Dflt_PIDSpdLeCfg(&pidCfg))
	{
		speedLeftPlant->Config->Factor_KP_scld = (int32_t)pidCfg.KP_scld;
		speedLeftPlant->Config->Factor_KI_scld = (int32_t)pidCfg.KI_scld;
		speedLeftPlant->Config->Factor_KD_scld = (int32_t)pidCfg.KD_scld;
		speedLeftPlant->Config->iWindUpMaxVal  = (int32_t)pidCfg.iWindupMaxVal;
		speedLeftPlant->Config->Scale 		  = (int32_t)pidCfg.Scale;
		NVM_Save_PIDSpdLeCfg(&pidCfg);
	}
	else
	{
		/* error handling */
	}

	if ( ERR_OK == NVM_Read_PIDSpdRiCfg(&pidCfg) )
	{
		speedRightPlant->Config->Factor_KP_scld = (int32_t)pidCfg.KP_scld;
		speedRightPlant->Config->Factor_KI_scld = (int32_t)pidCfg.KI_scld;
		speedRightPlant->Config->Factor_KD_scld = (int32_t)pidCfg.KD_scld;
		speedRightPlant->Config->iWindUpMaxVal  = (int32_t)pidCfg.iWindupMaxVal;
		speedRightPlant->Config->Scale 		    = (int32_t)pidCfg.Scale;
	}
	else if(ERR_OK == NVM_Read_Dflt_PIDSpdRiCfg(&pidCfg))
	{
		speedRightPlant->Config->Factor_KP_scld = (int32_t)pidCfg.KP_scld;
		speedRightPlant->Config->Factor_KI_scld = (int32_t)pidCfg.KI_scld;
		speedRightPlant->Config->Factor_KD_scld = (int32_t)pidCfg.KD_scld;
		speedRightPlant->Config->iWindUpMaxVal  = (int32_t)pidCfg.iWindupMaxVal;
		speedRightPlant->Config->Scale 		   = (int32_t)pidCfg.Scale;
		NVM_Save_PIDSpdRiCfg(&pidCfg);
	}
	else
	{
		/* error handling */
	}
}



#ifdef MASTER_pid_C_
#undef MASTER_pid_C_
#endif /* !MASTER_pid_C_ */

