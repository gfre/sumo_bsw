/*******************************************************************************
 * @brief 	Main state machine of application software layer.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date 	13.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements an (extended) state machine for the Sumo robot. It
 * handles the main states of the application software layer.
 *
 * ==============================================================================
 */

#define MASTER_STATE_C_

#include "Platform.h"
#include "CLS1.h"
#include "RNET1.h"
#include "state.h"
#include "stud.h"



static uint8_t STATE_PrintHelp(const CLS1_StdIOType *io_);
static StdRtnType STATE_PrintStatus(const CLS1_StdIOType *io_);
static const uint8 * STATE_ReadStateString(mainState_t state_);
static void STATE_RunStateMachine(void);

static mainState_t mainState = MAIN_STATE_STARTUP;




static void STATE_RunStateMachine(void) {
	switch (mainState) {
	case MAIN_STATE_STARTUP:
		mainState = MAIN_STATE_INIT;
		break;

	case MAIN_STATE_INIT:
		STUD_Init();
		RNET1_PowerUp();
		mainState = MAIN_STATE_IDLE;
		break;

	case MAIN_STATE_IDLE:
		mainState = MAIN_STATE_NORMAL;
		break;

	case MAIN_STATE_NORMAL:
		STUD_Main();
		break;

	default:
	case MAIN_STATE_ERROR:
			break;

	}/* switch */
	return;
}


static uint8_t STATE_PrintHelp(const CLS1_StdIOType *io_) {
	CLS1_SendHelpStr((unsigned char*)"state", (unsigned char*)"Group of state commands\r\n", io_->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows state help or status\r\n", io_->stdOut);
	return ERR_OK;
}

static StdRtnType STATE_PrintStatus(const CLS1_StdIOType *io_) {
	uint8 retVal = RTN_OK;
	CLS1_SendStatusStr((unsigned char*)"state", (unsigned char*)"\r\n", io_->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  current", STATE_ReadStateString(mainState) , io_->stdOut);
	return retVal;
}

static const uint8 * STATE_ReadStateString(mainState_t state_){
	switch(state_) {
		case MAIN_STATE_STARTUP: return "STARTUP\r\n";
		case MAIN_STATE_INIT:    return "INIT\r\n";
		case MAIN_STATE_IDLE:    return "IDLE\r\n";
		case MAIN_STATE_NORMAL:  return "NORMAL\r\n";
		case MAIN_STATE_ERROR:   return "ERROR\r\n";
		default: return ">> fatal error - unknown or undefined main state <<\r\n";
	}
}

void STATE_mainFct(void){
	STATE_RunStateMachine();
}


uint8 STATE_ParseCommand(const unsigned char *cmd_, bool *handled, const CLS1_StdIOType *io_) {
	uint8 res = ERR_OK;
	if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd_, (char*)"state help")==0) {
		*handled = TRUE;
		return STATE_PrintHelp(io_);
	} else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd_, (char*)"state status")==0) {
		*handled = TRUE;
		return STATE_PrintStatus(io_);
	}
	return res;
}


#ifdef MASTER_STATE_C_
#undef MASTER_STATE_C_
#endif
