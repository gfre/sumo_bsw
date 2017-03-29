/***************************************************************************************************
  * @brief 	Command line shell handler of the software component of the DC motors.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	29.03.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module handles the interface between the software component of the DC motors
 * and the command line shell CLS.
 * 
 *==================================================================================================
 */

#define MASTER_mot_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "mot_clshdlr.h"
#include "UTIL1.h"
#include "mot.h"

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void MOT_PrintHelp(const CLS1_StdIOType *io);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
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

	CLS1_SendStatusStr((unsigned char*)"  on/off", MOT_Get_IsMotorOn()?(unsigned char*)"on\r\n":(unsigned char*)"off\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  motor L", (unsigned char*)"", io->stdOut);
	buf[0] = '\0';

	UTIL1_Num16sToStrFormatted(buf, sizeof(buf), (int16_t)MOT_GetMotorHandle(MOT_MOTOR_LEFT)->currSpeedPercent, ' ', 4);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"% 0x");
	UTIL1_strcatNum16Hex(buf, sizeof(buf), MOT_GetVal(MOT_GetMotorHandle(MOT_MOTOR_LEFT)));
	UTIL1_strcat(buf, sizeof(buf),(unsigned char*)(MOT_GetDirection(MOT_GetMotorHandle(MOT_MOTOR_LEFT))==MOT_DIR_FORWARD?", fw":", bw"));

	CLS1_SendStr(buf, io->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

	CLS1_SendStatusStr((unsigned char*)"  motor R", (unsigned char*)"", io->stdOut);
	buf[0] = '\0';
	UTIL1_Num16sToStrFormatted(buf, sizeof(buf), (int16_t)MOT_GetMotorHandle(MOT_MOTOR_RIGHT)->currSpeedPercent, ' ', 4);
	UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"% 0x");
	UTIL1_strcatNum16Hex(buf, sizeof(buf), MOT_GetVal(MOT_GetMotorHandle(MOT_MOTOR_RIGHT)));
	UTIL1_strcat(buf, sizeof(buf),(unsigned char*)(MOT_GetDirection(MOT_GetMotorHandle(MOT_MOTOR_RIGHT))==MOT_DIR_FORWARD?", fw":", bw"));

	CLS1_SendStr(buf, io->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
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
		MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_LEFT), MOT_DIR_FORWARD);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"motor R forward")==0) {
		MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), MOT_DIR_FORWARD);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"motor L backward")==0) {
		MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_LEFT), MOT_DIR_BACKWARD);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"motor R backward")==0) {
		MOT_SetDirection(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), MOT_DIR_BACKWARD);
		*handled = TRUE;
	} else if (UTIL1_strncmp((char*)cmd, (char*)"motor L duty ", sizeof("motor L duty ")-1)==0) {
		if (!MOT_Get_IsMotorOn()) {
			CLS1_SendStr((unsigned char*)"Motor is OFF, cannot set duty.\r\n", io->stdErr);
			res = ERR_FAILED;
		} else {
			p = cmd+sizeof("motor L duty");
			if (UTIL1_xatoi(&p, &val)==ERR_OK && val >=-100 && val<=100) {
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), (MOT_SpeedPercent)val);
				*handled = TRUE;
			} else {
				CLS1_SendStr((unsigned char*)"Wrong argument, must be in the range -100..100\r\n", io->stdErr);
				res = ERR_FAILED;
			}
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"motor R duty ", sizeof("motor R duty ")-1)==0) {
		if (!MOT_Get_IsMotorOn()) {
			CLS1_SendStr((unsigned char*)"Motor is OFF, cannot set duty.\r\n", io->stdErr);
			res = ERR_FAILED;
		} else {
			p = cmd+sizeof("motor R duty");
			if (UTIL1_xatoi(&p, &val)==ERR_OK && val >=-100 && val<=100) {
				MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), (MOT_SpeedPercent)val);
				*handled = TRUE;
			} else {
				CLS1_SendStr((unsigned char*)"Wrong argument, must be in the range -100..100\r\n", io->stdErr);
				res = ERR_FAILED;
			}
		}
	} else if (UTIL1_strncmp((char*)cmd, (char*)"motor on", sizeof("motor on")-1)==0) {
		 MOT_OnOff(TRUE);
		*handled = TRUE;
	} else if (UTIL1_strncmp((char*)cmd, (char*)"motor off", sizeof("motor off")-1)==0) {
		MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0);
		MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0);
		 MOT_OnOff(FALSE);
		*handled = TRUE;

	}
	return res;
}




#ifdef MASTER_mot_clshdlr_C_
#undef MASTER_mot_clshdlr_C_
#endif /* !MASTER_mot_clshdlr_C_ */
