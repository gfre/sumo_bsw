/***********************************************************************************************//**
 * @file		buz_clshdlr.c
 * @ingroup		buz
 * @brief 		Implementation of the command line shell handler for the SWC @a Buzzer
 *
 * This module implements the interface from the SWC @a Buzzer (@b BUZ) to the SWC Shell (@b SH).
 * It introduces application specific commands for requesting buzzer help and status information
 * via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.04.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_buz_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "buz_clshdlr.h"
#include "buz_api.h"


/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t BUZ_PrintHelp(const CLS1_StdIOType *io_);
static uint8_t BUZ_PrintStatus(const CLS1_StdIOType *io_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t BUZ_PrintHelp(const CLS1_StdIOType *io_)
{
  CLS1_SendHelpStr((unsigned char*)"buzzer", (unsigned char*)"Group of buzzer commands\r\n", io_->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows radio help or status\r\n", io_->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  buz <freq> <time>", (unsigned char*)"Beep for time (ms) and frequency (kHz)\r\n", io_->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  play tune", (unsigned char*)"Play tune\r\n", io_->stdOut);
  return ERR_OK;
}

static uint8_t BUZ_PrintStatus(const CLS1_StdIOType *io_)
{
  (void)io_; /* not used */
  return ERR_OK;
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t BUZ_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
	const unsigned char *p;
	uint16_t freq, duration;

	if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd_, (char*)"buzzer help")==0) {
		*handled_ = TRUE;
		return BUZ_PrintHelp(io_);
	} else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd_, (char*)"buzzer status")==0) {
		*handled_ = TRUE;
		return BUZ_PrintStatus(io_);
	} else if (UTIL1_strncmp((char*)cmd_, (char*)"buzzer buz ", sizeof("buzzer buz ")-1)==0) {
		*handled_ = TRUE;
		p = cmd_+sizeof("buzzer buz ")-1;
		if (UTIL1_ScanDecimal16uNumber(&p, &freq)==ERR_OK && UTIL1_ScanDecimal16uNumber(&p, &duration)==ERR_OK) {
			if (BUZ_Beep(freq, duration)!=ERR_OK) {
				CLS1_SendStr((unsigned char*)"Starting buzzer failed\r\n", io_->stdErr);
				return ERR_FAILED;
			}
			return ERR_OK;
		}
	} else if (UTIL1_strcmp((char*)cmd_, (char*)"buzzer play tune")==0) {
		*handled_ = TRUE;
		return BUZ_PlayTune(BUZ_TUNE_WELCOME);
	}
	return ERR_OK;
}



#ifdef MASTER_buz_clshdlr_C_
#undef MASTER_buz_clshdlr_C_
#endif /* !MASTER_buz_clshdlr_C_ */
