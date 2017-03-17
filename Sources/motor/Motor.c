/*******************************************************************************
 * @brief 	Motor driver implementation.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 		06.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module a driver for up to two small DC motors.
 *
 * ==============================================================================
 */

#include "Motor.h"
#include "DIRR.h"
#include "DIRL.h"
#include "PWMR.h"
#include "PWML.h"
#include "UTIL1.h"


static MOT_MotorDevice motorL, motorR;
static bool isMotorOn = TRUE;

MOT_MotorDevice *MOT_GetMotorHandle(MOT_MotorSide side) {
	if (side==MOT_MOTOR_LEFT) {
		return &motorL;
	} else {
		return &motorR;
	}
}

static uint8_t PWMLSetRatio16(uint16_t ratio) {
	return PWML_SetRatio16(ratio);
}

static uint8_t PWMRSetRatio16(uint16_t ratio) {
	return PWMR_SetRatio16(ratio);
}

static void DirLPutVal(bool val) {
	DIRL_PutVal(val);
}

static void DirRPutVal(bool val) {
	DIRR_PutVal(val);
}


void MOT_SetVal(MOT_MotorDevice *motor, uint16_t val) {
	if (isMotorOn) {
		motor->currPWMvalue = val;
		(void)motor->SetRatio16(val);
	} else { /* have motor stopped */
		motor->currPWMvalue = 0xFFFF;
		(void)motor->SetRatio16(0xFFFF);
	}
}

void MOT_OnOff(bool on) {
	isMotorOn = on;
	if (!on) {
		MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0);
		MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0);
	}
}

uint16_t MOT_GetVal(MOT_MotorDevice *motor) {
	return motor->currPWMvalue;
}

void MOT_Invert(MOT_MotorDevice *motor, bool inverted) {
	motor->inverted = inverted;
}

void MOT_SetSpeedPercent(MOT_MotorDevice *motor, MOT_SpeedPercent percent) {
	uint32_t val;

	if (percent>100) { /* make sure we are within 0..100 */
		percent = 100;
	} else if (percent<-100) {
		percent = -100;
	}
	motor->currSpeedPercent = percent; /* store value */
	if (percent<0) {
		MOT_SetDirection(motor, MOT_DIR_BACKWARD);
		percent = -percent; /* make it positive */
	} else {
		MOT_SetDirection(motor, MOT_DIR_FORWARD);
	}
	val = ((100-percent)*0xffff)/100; /* H-Bridge is low active! */
	MOT_SetVal(motor, (uint16_t)val);
}

void MOT_UpdatePercent(MOT_MotorDevice *motor, MOT_Direction dir) {
	motor->currSpeedPercent = ((0xffff-motor->currPWMvalue)*100)/0xffff;
	if (dir==MOT_DIR_BACKWARD) {
		motor->currSpeedPercent = -motor->currSpeedPercent;
	}
}

MOT_SpeedPercent MOT_GetSpeedPercent(MOT_MotorDevice *motor) {
	return motor->currSpeedPercent;
}

void MOT_ChangeSpeedPercent(MOT_MotorDevice *motor, MOT_SpeedPercent relPercent) {
	relPercent += motor->currSpeedPercent; /* make absolute number */
	if (relPercent>100) { /* check for overflow */
		relPercent = 100;
	} else if (relPercent<-100) { /* and underflow */
		relPercent = -100;
	}
	MOT_SetSpeedPercent(motor, relPercent);  /* set speed. This will care about the direction too */
}

void MOT_SetDirection(MOT_MotorDevice *motor, MOT_Direction dir) {
	if (dir==MOT_DIR_FORWARD ) {
		motor->DirPutVal(motor->inverted?0:1);
		if (motor->currSpeedPercent<0) {
			motor->currSpeedPercent = -motor->currSpeedPercent;
		}
	} else if (dir==MOT_DIR_BACKWARD) {
		motor->DirPutVal(motor->inverted?1:0);
		if (motor->currSpeedPercent>0) {
			motor->currSpeedPercent = -motor->currSpeedPercent;
		}
	}
}

MOT_Direction MOT_GetDirection(MOT_MotorDevice *motor) {
	if (motor->currSpeedPercent<0) {
		return MOT_DIR_BACKWARD;
	} else {
		return MOT_DIR_FORWARD;
	}
}

#if 1
static void MOT_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*)"motor", (unsigned char*)"Group of motor commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows motor help or status\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  on|off", (unsigned char*)"Enables or disables motor\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  (L|R) forward|backward", (unsigned char*)"Change motor direction\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  (L|R) duty <number>", (unsigned char*)"Change motor PWM (-100..+100)%\r\n", io->stdOut);
}

static void MOT_PrintStatus(const CLS1_StdIOType *io) {
	unsigned char buf[32];

	CLS1_SendStatusStr((unsigned char*)"Motor", (unsigned char*)"\r\n", io->stdOut);

	CLS1_SendStatusStr((unsigned char*)"  inverted L", MOT_GetMotorHandle(MOT_MOTOR_LEFT)->inverted?(unsigned char*)"yes\r\n":(unsigned char*)"no\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  inverted R", MOT_GetMotorHandle(MOT_MOTOR_RIGHT)->inverted?(unsigned char*)"yes\r\n":(unsigned char*)"no\r\n", io->stdOut);

	CLS1_SendStatusStr((unsigned char*)"  on/off", isMotorOn?(unsigned char*)"on\r\n":(unsigned char*)"off\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  motor L", (unsigned char*)"", io->stdOut);
	buf[0] = '\0';
	UTIL1_Num16sToStrFormatted(buf, sizeof(buf), (int16_t)motorL.currSpeedPercent, ' ', 4);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"% 0x");
	UTIL1_strcatNum16Hex(buf, sizeof(buf), MOT_GetVal(&motorL));
	UTIL1_strcat(buf, sizeof(buf),(unsigned char*)(MOT_GetDirection(&motorL)==MOT_DIR_FORWARD?", fw":", bw"));

	CLS1_SendStr(buf, io->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

	CLS1_SendStatusStr((unsigned char*)"  motor R", (unsigned char*)"", io->stdOut);
	buf[0] = '\0';
	UTIL1_Num16sToStrFormatted(buf, sizeof(buf), (int16_t)motorR.currSpeedPercent, ' ', 4);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"% 0x");
	UTIL1_strcatNum16Hex(buf, sizeof(buf), MOT_GetVal(&motorR));
	UTIL1_strcat(buf, sizeof(buf),(unsigned char*)(MOT_GetDirection(&motorR)==MOT_DIR_FORWARD?", fw":", bw"));

	CLS1_SendStr(buf, io->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

}

uint8_t MOT_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	uint8_t res = ERR_OK;
	int32_t val;
	const unsigned char *p;
	unsigned char buf[32];

	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"motor help")==0) {
		MOT_PrintHelp(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"motor status")==0) {
		MOT_PrintStatus(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"motor L forward")==0) {
		MOT_SetDirection(&motorL, MOT_DIR_FORWARD);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"motor R forward")==0) {
		MOT_SetDirection(&motorR, MOT_DIR_FORWARD);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"motor L backward")==0) {
		MOT_SetDirection(&motorL, MOT_DIR_BACKWARD);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"motor R backward")==0) {
		MOT_SetDirection(&motorR, MOT_DIR_BACKWARD);
		*handled = TRUE;
	} else if (UTIL1_strncmp((char*)cmd, (char*)"motor L duty ", sizeof("motor L duty ")-1)==0) {
		if (!isMotorOn) {
			CLS1_SendStr((unsigned char*)"Motor is OFF, cannot set duty.\r\n", io->stdErr);
			res = ERR_FAILED;
		} else {
			p = cmd+sizeof("motor L duty");
			if (UTIL1_xatoi(&p, &val)==ERR_OK && val >=-100 && val<=100) {
				MOT_SetSpeedPercent(&motorL, (MOT_SpeedPercent)val);
				*handled = TRUE;
			} else {
				CLS1_SendStr((unsigned char*)"Wrong argument, must be in the range -100..100\r\n", io->stdErr);
				res = ERR_FAILED;
			}
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"motor R duty ", sizeof("motor R duty ")-1)==0) {
		if (!isMotorOn) {
			CLS1_SendStr((unsigned char*)"Motor is OFF, cannot set duty.\r\n", io->stdErr);
			res = ERR_FAILED;
		} else {
			p = cmd+sizeof("motor R duty");
			if (UTIL1_xatoi(&p, &val)==ERR_OK && val >=-100 && val<=100) {
				MOT_SetSpeedPercent(&motorR, (MOT_SpeedPercent)val);
				*handled = TRUE;
			} else {
				CLS1_SendStr((unsigned char*)"Wrong argument, must be in the range -100..100\r\n", io->stdErr);
				res = ERR_FAILED;
			}
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"motor on", sizeof("motor on")-1)==0) {
		isMotorOn = TRUE;
		*handled = TRUE;
	} else if (UTIL1_strncmp((char*)cmd, (char*)"motor off", sizeof("motor off")-1)==0) {
		MOT_SetSpeedPercent(&motorL, 0);
		MOT_SetSpeedPercent(&motorR, 0);
		isMotorOn = FALSE;
		*handled = TRUE;

	}
	return res;
}
#endif /* PL_CONFIG_HAS_SHELL */

void MOT_Deinit(void) {
	/* nothig needed for now */
}

void MOT_Init(void) {
	motorL.inverted = CAU_SUMO_PLT_MOTOR_LEFT_INVERTED;
	motorR.inverted = CAU_SUMO_PLT_MOTOR_RIGHT_INVERTED;
	motorL.DirPutVal = DirLPutVal;
	motorR.DirPutVal = DirRPutVal;
	motorL.SetRatio16 = PWMLSetRatio16;
	motorR.SetRatio16 = PWMRSetRatio16;
	MOT_SetSpeedPercent(&motorL, 0);
	MOT_SetSpeedPercent(&motorR, 0);
	(void)PWML_Enable();
	(void)PWMR_Enable();
}

