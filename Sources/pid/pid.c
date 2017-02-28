/*******************************************************************************
 * @brief 	PID controller implementation.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 		06.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#include "Platform.h"
#include "Pid.h"
#include "Motor.h"
#include "UTIL1.h"
//#include "Reflectance.h"

#define PID_DEBUG 0 /* careful: this will slow down the PID loop frequency! */

static PID_Config posLeftConfig = {0u};
static PID_Config posRightConfig = {0u};
static PID_Config speedLeftConfig = {0u};
static PID_Config speedRightConfig = {0u};

static int32_t PID(int32_t currVal, int32_t setVal, PID_Config *config) {
	int32_t error;
	int32_t pid;

	/* perform PID closed control loop calculation */
	error = setVal-currVal; /* calculate error */
	pid = (error*config->pFactor100)/100; /* P part */
	config->integral += error; /* integrate error */
	if (config->integral>config->iAntiWindup) {
		config->integral = config->iAntiWindup;
	} else if (config->integral<-config->iAntiWindup) {
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




static void PID_PosCfg(int32_t currPos, int32_t setPos, bool isLeft, PID_Config *config) {
	int32_t speed;
	MOT_Direction direction=MOT_DIR_FORWARD;
	MOT_MotorDevice *motHandle;

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

void PID_Pos(int32_t currPos, int32_t setPos, bool isLeft) {
	if (isLeft) {
		PID_PosCfg(currPos, setPos, isLeft, &posLeftConfig);
	} else {
		PID_PosCfg(currPos, setPos, isLeft, &posRightConfig);
	}
}

static void PID_SpeedCfg(int32_t currSpeed, int32_t setSpeed, bool isLeft, PID_Config *config) {
	int32_t speed;
	MOT_Direction direction=MOT_DIR_FORWARD;
	MOT_MotorDevice *motHandle;

	if (setSpeed==0) {
		speed = 0;
	} else {
		speed = PID(currSpeed, setSpeed, config);
	}
	if (speed>=0) {
		direction = MOT_DIR_FORWARD;
	} else { /* negative, make it positive */
		speed = -speed; /* make positive */
		direction = MOT_DIR_BACKWARD;
	}
	/* speed shall be positive here, make sure it is within 16bit PWM boundary */
	if (speed>0xFFFF) {
		speed = 0xFFFF;
	}
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

void PID_Speed(int32_t currSpeed, int32_t setSpeed, bool isLeft) {
	if (isLeft) {
		PID_SpeedCfg(currSpeed, setSpeed, isLeft, &speedLeftConfig);
	} else {
		PID_SpeedCfg(currSpeed, setSpeed, isLeft, &speedRightConfig);
	}
}



void PID_Start(void) {
	posLeftConfig.lastError = 0;
	posLeftConfig.integral = 0;
	posRightConfig.lastError = 0;
	posRightConfig.integral = 0;
	speedLeftConfig.lastError = 0;
	speedLeftConfig.integral = 0;
	speedRightConfig.lastError = 0;
	speedRightConfig.integral = 0;
}

void PID_Deinit(void) {
}



void PID_Init(void)
{


	posLeftConfig.pFactor100 = 1000;
	posLeftConfig.iFactor100 = 1;
	posLeftConfig.dFactor100 = 50;
	posLeftConfig.iAntiWindup = 200;
	posLeftConfig.maxSpeedPercent = 100;
	posLeftConfig.lastError = 0;
	posLeftConfig.integral = 0;
	posRightConfig.pFactor100 = posLeftConfig.pFactor100;
	posRightConfig.iFactor100 = posLeftConfig.iFactor100;
	posRightConfig.dFactor100 = posLeftConfig.dFactor100;
	posRightConfig.iAntiWindup = posLeftConfig.iAntiWindup;
	posRightConfig.maxSpeedPercent = posLeftConfig.maxSpeedPercent;
	posRightConfig.lastError = posLeftConfig.lastError;
	posRightConfig.integral = posLeftConfig.integral;
	speedLeftConfig.pFactor100 = 2000;
	speedLeftConfig.iFactor100 = 80;
	speedLeftConfig.dFactor100 = 0;
	speedLeftConfig.iAntiWindup = 120000;
	speedLeftConfig.maxSpeedPercent = 100;
	speedLeftConfig.lastError = 0;
	speedLeftConfig.integral = 0;
	speedRightConfig.pFactor100 = speedLeftConfig.pFactor100;
	speedRightConfig.iFactor100 = speedLeftConfig.iFactor100;
	speedRightConfig.dFactor100 = speedLeftConfig.dFactor100;
	speedRightConfig.iAntiWindup = speedLeftConfig.iAntiWindup;
	speedRightConfig.maxSpeedPercent = speedLeftConfig.maxSpeedPercent;
	speedRightConfig.lastError = speedLeftConfig.lastError;
	speedRightConfig.integral = speedLeftConfig.integral;
}


PID_Config *PID_Get_PosLeCfg(void) { return &posLeftConfig; }
PID_Config *PID_Get_PosRiCfg(void) { return &posRightConfig; }
PID_Config *PID_Get_SpdLeCfg(void) { return &speedLeftConfig; }
PID_Config *PID_Get_SpdRiCfg(void) { return &speedRightConfig; }
