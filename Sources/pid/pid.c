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
#define MAXVAL (0xFFFF)

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static int32_t PID(int32_t currVal, int32_t setVal, PID_Config_t *config);
static void PID_PosCfg(int32_t currPos, int32_t setPos, bool isLeft, PID_Config_t *config);
static void PID_SpeedCfg(int32_t currSpeed, int32_t setSpeed, bool isLeft, PID_Config_t *config);
static int32_t PI(PID_Plant_t* plant_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static PID_Config_t posLeftConfig = {0u};
static PID_Config_t posRightConfig = {0u};
static PID_Config_t speedLeftConfig = {0u};
static PID_Config_t speedRightConfig = {0u};

static PID_Plant_t pActivePlant = {0};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static int32_t PID(int32_t currVal, int32_t setVal, PID_Config_t *config) {
	int32_t error;
	int32_t pid;

	/* perform PID closed control loop calculation */
	error = setVal-currVal; /* calculate error */
	pid = (error*config->pFactor100)/100; /* P part */
	config->integral += error; /* integrate error */
	if (config->integral > config->iAntiWindup) {
		config->integral = config->iAntiWindup;
	} else if (config->integral < -config->iAntiWindup) {
		config->integral = -config->iAntiWindup;
	}
#if 1 /* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/ */
	{
		int32_t max;

		max = 0xffff; /* max value of PWM */
		if (config->integral > max) {
			config->integral = max;
		} else if (config->integral < -max) {
			config->integral = -max;
		}
	}
#endif
	pid += (config->integral*config->iFactor100)/100; /* add I part */
	pid += ((error-config->lastError)*config->dFactor100)/100; /* add D part */
	config->lastError = error; /* remember for next iteration of D part */
	return pid;
}


static int32_t PI(PID_Plant_t* plant_)
{
	int32_t pTerm = 0;
	int32_t pi    = 0;
	int32_t error = 0;

	error = plant_->pGetTargetVal() - plant_->pGetCurrentVal();

	if( (plant_->Saturation < 0  && error < 0) || (plant_->Saturation > 0 && error > 0) )
	{
		/* don't allow integrating in direction of saturation -> do nothing */
	}
	else //allow integrating in opposite direction to saturation
	{
	plant_->integralVal += (plant_->Factor_KI_scld*error)/plant_->Scale;
	}

	if( (plant_->integralVal > -(plant_->iWindUpMaxVal)) && (plant_->integralVal < plant_->iWindUpMaxVal) )
	{
		plant_->Saturation = 0;
	}
	else if(plant_->integralVal < -(plant_->iWindUpMaxVal))
	{
		plant_->integralVal = -(plant_->iWindUpMaxVal);
		plant_->Saturation  = -1;
	}
	else if(plant_->integralVal > plant_->iWindUpMaxVal)
	{
		plant_->integralVal = plant_->iWindUpMaxVal;
		plant_->Saturation   = 1;
	}

	pTerm = (plant_->Factor_KP_scld)*error/plant_->Scale;

	if(pTerm < -(plant_->iWindUpMaxVal))     pTerm = -(plant_->iWindUpMaxVal);
	else if(pTerm > (plant_->iWindUpMaxVal)) pTerm =  (plant_->iWindUpMaxVal);

	pi = pTerm + plant_->integralVal;
	if(pi < -(plant_->iWindUpMaxVal))     pi = -(int32_t)MAXVAL;
	else if(pi > (plant_->iWindUpMaxVal)) pi =  (int32_t)MAXVAL;

	return pi;
}






static void PID_PosCfg(int32_t currPos, int32_t setPos, bool isLeft, PID_Config_t *config) {
	int32_t speed;
	MOT_Direction_t direction = MOT_DIR_FORWARD;
	MOT_MotorDevice_t *motHandle;

	int error;

	error = setPos-currPos;
	if (error>-10 && error<10) { /* avoid jitter around zero */
		setPos = currPos;
	}
	speed = PID(currPos, setPos, config);
	/* transform into motor speed */
	speed *= 1000; /* scale PID, otherwise we need high PID constants */
	if (speed>=0) {
		direction = MOT_DIR_FORWARD;
	} else { /* negative, make it positive */
		speed = -speed; /* make positive */
		direction = MOT_DIR_BACKWARD;
	}
	/* speed is now always positive, make sure it is within 16bit PWM boundary */
	if (speed>0xFFFF) {
		speed = 0xFFFF;
	}
	/* limit speed to maximum value */
	speed = (speed*config->maxSpeedPercent)/100;
	/* send new speed values to motor */
	if (isLeft) {
		motHandle = MOT_GetMotorHandle(MOT_MOTOR_LEFT);
	} else {
		motHandle = MOT_GetMotorHandle(MOT_MOTOR_RIGHT);
	}
	MOT_SetVal(motHandle, 0xFFFF-speed); /* PWM is low active */
	MOT_SetDirection(motHandle, direction);
	MOT_UpdatePercent(motHandle, direction);
}


static void PID_SpeedCfg(int32_t currSpeed, int32_t setSpeed, bool isLeft, PID_Config_t *config) {
	int32_t speed;
	MOT_Direction_t direction = MOT_DIR_FORWARD;
	MOT_MotorDevice_t *motHandle;

	/*if (setSpeed==0) {
		speed = 0;
		PID_Start();
	}
	else */
	{
		speed = PID(currSpeed, setSpeed, config);
	}
	if (speed>=0) {
		direction = MOT_DIR_FORWARD;
	} else { /* negative, make it positive */
		speed = -speed; /* make positive */
		direction = MOT_DIR_BACKWARD;
	}
//	/* speed shall be positive here, make sure it is within 16bit PWM boundary */
//	if (speed>0xFFFF) {
//		speed = 0xFFFF;
//	}
	/* send new speed values to motor */
	if (isLeft) {
		motHandle = MOT_GetMotorHandle(MOT_MOTOR_LEFT);
	} else {
		motHandle = MOT_GetMotorHandle(MOT_MOTOR_RIGHT);
	}
	MOT_SetVal(motHandle, 0xFFFF-speed); /* PWM is low active */
	MOT_SetDirection(motHandle, direction);
	MOT_UpdatePercent(motHandle, direction);
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void PID_Pos(int32_t currPos, int32_t setPos, bool isLeft) {
	if (isLeft) {
		PID_PosCfg(currPos, setPos, isLeft, &posLeftConfig);
	} else {
		PID_PosCfg(currPos, setPos, isLeft, &posRightConfig);
	}
}



void PID_Speed(int32_t currSpeed, int32_t setSpeed, bool isLeft) {
	if (isLeft) {
		PID_SpeedCfg(currSpeed, setSpeed, isLeft, &speedLeftConfig);
	} else {
		PID_SpeedCfg(currSpeed, setSpeed, isLeft, &speedRightConfig);
	}
}



void PID_Start(void)
{
	posLeftConfig.lastError = 0;
	posLeftConfig.integral = 0;
	posRightConfig.lastError = 0;
	posRightConfig.integral = 0;
	speedLeftConfig.lastError = 0;
	speedLeftConfig.integral = 0;
	speedRightConfig.lastError = 0;
	speedRightConfig.integral = 0;
}

void PID_Deinit(void)
{
	/* nothing to do */
}



void PID_Init(void)
{
	NVM_PidCfg_t pidCfg = {0u};
	if ( ERR_OK == NVM_Read_PIDPosCfg(&pidCfg) )
	{
		posLeftConfig.pFactor100 = (int32_t)pidCfg.pGain100;
		posLeftConfig.iFactor100 = (int32_t)pidCfg.iGain100;
		posLeftConfig.dFactor100 = (int32_t)pidCfg.dGain100;
		posLeftConfig.iAntiWindup = (int32_t)pidCfg.iAntiWindup;
		posLeftConfig.maxSpeedPercent = (int32_t)pidCfg.maxSpdPerc;
	}
	else if(ERR_OK == NVM_Read_Dflt_PIDPosCfg(&pidCfg))
	{
		posLeftConfig.pFactor100 = (int32_t)pidCfg.pGain100;
		posLeftConfig.iFactor100 = (int32_t)pidCfg.iGain100;
		posLeftConfig.dFactor100 = (int32_t)pidCfg.dGain100;
		posLeftConfig.iAntiWindup = (int32_t)pidCfg.iAntiWindup;
		posLeftConfig.maxSpeedPercent = (int32_t)pidCfg.maxSpdPerc;
		NVM_Save_PIDPosCfg(&pidCfg);
	}
	else
	{
		/* error handling */
	}
	posLeftConfig.lastError = 0;
	posLeftConfig.integral = 0;


	posRightConfig.pFactor100 = posLeftConfig.pFactor100;
	posRightConfig.iFactor100 = posLeftConfig.iFactor100;
	posRightConfig.dFactor100 = posLeftConfig.dFactor100;
	posRightConfig.iAntiWindup = posLeftConfig.iAntiWindup;
	posRightConfig.maxSpeedPercent = posLeftConfig.maxSpeedPercent;
	posRightConfig.lastError = posLeftConfig.lastError;
	posRightConfig.integral = posLeftConfig.integral;


	if ( ERR_OK == NVM_Read_PIDSpdLeCfg(&pidCfg) )
	{
		speedLeftConfig.pFactor100 = (int32_t)pidCfg.pGain100;
		speedLeftConfig.iFactor100 = (int32_t)pidCfg.iGain100;
		speedLeftConfig.dFactor100 = (int32_t)pidCfg.dGain100;
		speedLeftConfig.iAntiWindup = (int32_t)pidCfg.iAntiWindup;
		speedLeftConfig.maxSpeedPercent = (int32_t)pidCfg.maxSpdPerc;
	}
	else if(ERR_OK == NVM_Read_Dflt_PIDSpdLeCfg(&pidCfg))
	{
		speedLeftConfig.pFactor100 = (int32_t)pidCfg.pGain100;
		speedLeftConfig.iFactor100 = (int32_t)pidCfg.iGain100;
		speedLeftConfig.dFactor100 = (int32_t)pidCfg.dGain100;
		speedLeftConfig.iAntiWindup = (int32_t)pidCfg.iAntiWindup;
		speedLeftConfig.maxSpeedPercent = (int32_t)pidCfg.maxSpdPerc;
		NVM_Save_PIDSpdLeCfg(&pidCfg);
	}
	else
	{
		/* error handling */
	}

	if ( ERR_OK == NVM_Read_PIDSpdRiCfg(&pidCfg) )
	{
		speedRightConfig.pFactor100 = (int32_t)pidCfg.pGain100;
		speedRightConfig.iFactor100 = (int32_t)pidCfg.iGain100;
		speedRightConfig.dFactor100 = (int32_t)pidCfg.dGain100;
		speedRightConfig.iAntiWindup = (int32_t)pidCfg.iAntiWindup;
		speedRightConfig.maxSpeedPercent = (int32_t)pidCfg.maxSpdPerc;
	}
	else if(ERR_OK == NVM_Read_Dflt_PIDSpdRiCfg(&pidCfg))
	{
		speedRightConfig.pFactor100 = (int32_t)pidCfg.pGain100;
		speedRightConfig.iFactor100 = (int32_t)pidCfg.iGain100;
		speedRightConfig.dFactor100 = (int32_t)pidCfg.dGain100;
		speedRightConfig.iAntiWindup = (int32_t)pidCfg.iAntiWindup;
		speedRightConfig.maxSpeedPercent = (int32_t)pidCfg.maxSpdPerc;
		NVM_Save_PIDSpdRiCfg(&pidCfg);
	}
	else
	{
		/* error handling */
	}
}

void PID_Init2(PID_Plant_t* plant_)
{
	plant_->Saturation  = 0;
	plant_->integralVal = 0;
	plant_->lastError   = 0;
}
void PID_Deinit2(PID_Plant_t* plant_)
{
	plant_->Saturation  = 0;
	plant_->integralVal = 0;
	plant_->lastError   = 0;
}

PID_Config_t *PID_Get_PosLeCfg(void) { return &posLeftConfig; }
PID_Config_t *PID_Get_PosRiCfg(void) { return &posRightConfig; }
PID_Config_t *PID_Get_SpdLeCfg(void) { return &speedLeftConfig; }
PID_Config_t *PID_Get_SpdRiCfg(void) { return &speedRightConfig; }

void PID_Set_PlantType(PID_PlantType type_)
{
	PID_PlantCfg_t *tbl = NULL;
	tbl = Get_pPidCfg();
	if( (NULL != pActivePlant) && (NULL != pActivePlant->pInitFct) )
	{
		if(TRUE == pActivePlant->isInitialized)
		{
			pActivePlant->pDeinitFct();
			pActivePlant->isInitialized = FALSE;
		}
	}

	pActivePlant = &(tbl->pPlantTbl[type_]);
	if( (NULL != pActivePlant) && (NULL != pActivePlant->pInitFct) )
	{
		if(FALSE == pActivePlant->isInitialized)
		{
			pActivePlant->pInitFct();
			pActivePlant->isInitialized = TRUE;
		}
	}
	else
	{
		/* TODO error handling */
	}
}

PID_PlantType_t PID_Get_PlantType(void)
{
	return pActivePlant->PlantType;
}



#ifdef MASTER_pid_C_
#undef MASTER_pid_C_
#endif /* !MASTER_pid_C_ */

