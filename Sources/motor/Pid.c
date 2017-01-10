/**
 * \file
 * \brief This is the implementation of the PID Module
 * \author Erich Styger, erich.styger@hslu.ch
 */

#include "Platform.h"
#include "Pid.h"
#include "Motor.h"
#include "UTIL1.h"
//#include "Reflectance.h"

#define PID_DEBUG 0 /* careful: this will slow down the PID loop frequency! */

static PID_Config posLeftConfig, posRightConfig;
static PID_Config speedLeftConfig, speedRightConfig;

uint8_t PID_GetPIDConfig(PID_ConfigType config, PID_Config **confP) {
	switch(config) {
	case PID_CONFIG_POS_LEFT:
		*confP = &posLeftConfig; break;
	case PID_CONFIG_POS_RIGHT:
		*confP = &posRightConfig; break;
	case PID_CONFIG_SPEED_LEFT:
		*confP = &speedLeftConfig; break;
	case PID_CONFIG_SPEED_RIGHT:
		*confP = &speedRightConfig; break;
	default:
		*confP = NULL;
		return ERR_FAILED;
	}
	return ERR_OK;
}

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

static void PID_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*)"pid", (unsigned char*)"Group of PID commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows PID help or status\r\n", io->stdOut);
#if PL_HAS_POS_PID
	CLS1_SendHelpStr((unsigned char*)"  pos (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup position value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  pos speed <value>", (unsigned char*)"Maximum speed % value\r\n", io->stdOut);
#endif
#if PL_HAS_SPEED_PID
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup position value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  speed (L|R) speed <value>", (unsigned char*)"Maximum speed % value\r\n", io->stdOut);
#endif
#if PL_HAS_LINE_PID
	CLS1_SendHelpStr((unsigned char*)"  fw (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup line value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  fw speed <value>", (unsigned char*)"Maximum speed % value\r\n", io->stdOut);
#endif
#if PL_GO_DEADEND_BW
	CLS1_SendHelpStr((unsigned char*)"  bw (p|i|d|w) <value>", (unsigned char*)"Sets P, I, D or anti-Windup backward value\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  bw speed <value>", (unsigned char*)"Maximum backward speed % value\r\n", io->stdOut);
#endif
}

static void PrintPIDstatus(PID_Config *config, const unsigned char *kindStr, const CLS1_StdIOType *io) {
	unsigned char buf[48];
	unsigned char kindBuf[16];

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" PID");
	UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"p: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), config->pFactor100);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" i: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), config->iFactor100);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)" d: ");
	UTIL1_strcatNum32s(buf, sizeof(buf), config->dFactor100);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" windup");
	UTIL1_Num32sToStr(buf, sizeof(buf), config->iAntiWindup);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" error");
	UTIL1_Num32sToStr(buf, sizeof(buf), config->lastError);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" integral");
	UTIL1_Num32sToStr(buf, sizeof(buf), config->integral);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);

	UTIL1_strcpy(kindBuf, sizeof(buf), (unsigned char*)"  ");
	UTIL1_strcat(kindBuf, sizeof(buf), kindStr);
	UTIL1_strcat(kindBuf, sizeof(buf), (unsigned char*)" speed");
	UTIL1_Num8uToStr(buf, sizeof(buf), config->maxSpeedPercent);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"%\r\n");
	CLS1_SendStatusStr(kindBuf, buf, io->stdOut);
}

static void PID_PrintStatus(const CLS1_StdIOType *io) {
	CLS1_SendStatusStr((unsigned char*)"pid", (unsigned char*)"\r\n", io->stdOut);
	PrintPIDstatus(&posLeftConfig, (unsigned char*)"pos L", io);
	PrintPIDstatus(&posRightConfig, (unsigned char*)"pos R", io);
	PrintPIDstatus(&speedLeftConfig, (unsigned char*)"speed L", io);
	PrintPIDstatus(&speedRightConfig, (unsigned char*)"speed R", io);
}

static uint8_t ParsePidParameter(PID_Config *config, const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	const unsigned char *p;
	uint32_t val32u;
	uint8_t val8u;
	uint8_t res = ERR_OK;

	if (UTIL1_strncmp((char*)cmd, (char*)"p ", sizeof("p ")-1)==0) {
		p = cmd+sizeof("p");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			config->pFactor100 = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"i ", sizeof("i ")-1)==0) {
		p = cmd+sizeof("i");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			config->iFactor100 = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"d ", sizeof("d ")-1)==0) {
		p = cmd+sizeof("d");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			config->dFactor100 = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"w ", sizeof("w ")-1)==0) {
		p = cmd+sizeof("w");
		if (UTIL1_ScanDecimal32uNumber(&p, &val32u)==ERR_OK) {
			config->iAntiWindup = val32u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"speed ", sizeof("speed ")-1)==0) {
		p = cmd+sizeof("speed");
		if (UTIL1_ScanDecimal8uNumber(&p, &val8u)==ERR_OK && val8u<=100) {
			config->maxSpeedPercent = val8u;
			*handled = TRUE;
		} else {
			CLS1_SendStr((unsigned char*)"Wrong argument\r\n", io->stdErr);
			res = ERR_FAILED;
		}
	}
	return res;
}

uint8_t PID_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	uint8_t res = ERR_OK;

	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"pid help")==0) {
		PID_PrintHelp(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"pid status")==0) {
		PID_PrintStatus(io);
		*handled = TRUE;
	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid pos ", sizeof("pid pos ")-1)==0) {
		res = ParsePidParameter(&posLeftConfig, cmd+sizeof("pid pos ")-1, handled, io);
		if (res==ERR_OK) {
			res = ParsePidParameter(&posRightConfig, cmd+sizeof("pid pos ")-1, handled, io);
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid speed L ", sizeof("pid speed L ")-1)==0) {
		res = ParsePidParameter(&speedLeftConfig, cmd+sizeof("pid speed L ")-1, handled, io);
	} else if (UTIL1_strncmp((char*)cmd, (char*)"pid speed R ", sizeof("pid speed R ")-1)==0) {
		res = ParsePidParameter(&speedRightConfig, cmd+sizeof("pid speed R ")-1, handled, io);
	}
	return res;
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

void PID_Init(void) {
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
