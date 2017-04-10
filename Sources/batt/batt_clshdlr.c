/***********************************************************************************************//**
 * @file		batt_clshdlr.c
 * @ingroup		batt
 * @brief 		Implementation of the command line shell handler for the SWC @a Application
 *
 * This module implements the interface from the SWC Battery (@b BATT) to the SWC Shell (@b SH).
 * It introduces application specific commands for requesting battery voltage information
 * via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.04.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_batt_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "batt_clshdlr.h"
#include "batt_api.h"


/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t BATT_PrintStatus(const CLS1_StdIOType *io_);
static uint8_t BATT_PrintHelp(const CLS1_StdIOType *io_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t BATT_PrintStatus(const CLS1_StdIOType *io_)
{
  uint8_t buf[32];
  uint16_t cv;

  CLS1_SendStatusStr((unsigned char*)"battery", (unsigned char*)"\r\n", io_->stdOut);
  buf[0] = '\0';
  if (BATT_MeasureBatteryVoltage(&cv)==ERR_OK) {
    UTIL1_strcatNum32sDotValue100(buf, sizeof(buf), cv);
    UTIL1_strcat(buf, sizeof(buf), (uint8_t*)" V\r\n");
  } else {
    UTIL1_strcat(buf, sizeof(buf), (uint8_t*)"ERROR\r\n");
  }
  CLS1_SendStatusStr((unsigned char*)"  Voltage", buf, io_->stdOut);
  return ERR_OK;
}

static uint8_t BATT_PrintHelp(const CLS1_StdIOType *io_)
{
  CLS1_SendHelpStr((unsigned char*)"battery", (unsigned char*)"Group of battery commands\r\n", io_->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows battery help or status\r\n", io_->stdOut);
  return ERR_OK;
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t BATT_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
  uint8_t res = ERR_OK;

  if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd_, (char*)"battery help")==0) {
    *handled_ = TRUE;
    return BATT_PrintHelp(io_);

  } else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd_, (char*)"battery status")==0) {
    *handled_ = TRUE;
    return BATT_PrintStatus(io_);
  }
  return res;
}



#ifdef MASTER_batt_clshdlr_C_
#undef MASTER_batt_clshdlr_C_
#endif /* !MASTER_batt_clshdlr_C_ */
