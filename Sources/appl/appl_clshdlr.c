/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file appl_clshdlr.c
 * 
 *==================================================================================================
 */

#define MASTER_appl_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "appl_clshdlr.h"
#include "appl_Types.h"


/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8 PrintHelp(const CLS1_StdIOType *io_);
static uint8 PrintStatus(const CLS1_StdIOType *io_);
static const uint8 * ReadStateString(APPL_State_t state_);
static const uint8 * ReadCmdString(APPL_Cmd_t state_);

/*=================================== >> GLOBAL VARIABLES << =====================================*/


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8 PrintHelp(const CLS1_StdIOType *io_) {
  CLS1_SendHelpStr((unsigned char*)"appl", (unsigned char*)"Group of application commands\r\n", io_->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows application help or status\r\n", io_->stdOut);
  return ERR_OK;
}

static uint8 PrintStatus(const CLS1_StdIOType *io_)
{
	CLS1_SendStatusStr((unsigned char*)"appl", (unsigned char*)"\r\n", io_->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  sm state", ReadStateString(APPL_Get_SmState()) , io_->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  sm cmd", ReadCmdString(APPL_Get_SmCmd()) , io_->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  nextState", ReadStateString(APPL_Get_NextState()) , io_->stdOut);
	return ERR_OK;
}

static const uint8 * ReadStateString(APPL_State_t state_){
  switch(state_)
  {
  	  case APPL_STATE_NONE: 	return "NO or INVALID STATE\r\n";
  	  case APPL_STATE_STARTUP:	return "STARTUP\r\n";
  	  case APPL_STATE_INIT:    	return "INIT\r\n";
  	  case APPL_STATE_IDLE:    	return "IDLE\r\n";
  	  case APPL_STATE_NORMAL:  	return "NORMAL\r\n";
  	  case APPL_STATE_DEBUG:  	return "DEBUG\r\n";
  	  case APPL_STATE_ERROR:   	return "ERROR\r\n";
  	  default: 					return ">> fatal error - unknown application state <<\r\n";
  }
}

static const uint8 * ReadCmdString(APPL_Cmd_t state_){
  switch(state_) {
  	case Run: 		return "Run\r\n";
    case Enter:    	return "Enter\r\n";
    case Exit:    	return "Exit\r\n";
    default: return ">> fatal error - invalid or unknown state machine command <<\r\n";
  }
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8 APPL_ParseCommand(const unsigned char *cmd_, bool *handled, const CLS1_StdIOType *io_) {
  uint8 res = ERR_OK;
  if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd_, (char*)"appl help")==0) {
    *handled = TRUE;
    return PrintHelp(io_);
  } else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd_, (char*)"appl status")==0) {
    *handled = TRUE;
    return PrintStatus(io_);
  }
  return res;
}



#ifdef MASTER_appl_clshdlr_C_
#undef MASTER_appl_clshdlr_C_
#endif /* !MASTER_appl_clshdlr_C_ */
