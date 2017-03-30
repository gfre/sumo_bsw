/***************************************************************************************************
  * @brief 	Command line shell handler of the software component of the tachometer.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	30.03.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module handles the interface between the software component of the tachometer
 * and the command line shell CLS.
 * 
 *==================================================================================================
 */

#define MASTER_tacho_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tacho_clshdlr.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
/*!
 * \brief Prints the system low power status
 * \param io I/O channel to use for printing status
 */
static void TACHO_PrintStatus(const CLS1_StdIOType *io) {
	CLS1_SendStatusStr((unsigned char*)"Tacho", (unsigned char*)"\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  L speed", (unsigned char*)"", io->stdOut);
	CLS1_SendNum32s(TACHO_GetSpeed(TRUE), io->stdOut);
	CLS1_SendStr((unsigned char*)" steps/sec\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  R speed", (unsigned char*)"", io->stdOut);
	CLS1_SendNum32s(TACHO_GetSpeed(FALSE), io->stdOut);
	CLS1_SendStr((unsigned char*)" steps/sec\r\n", io->stdOut);
}

/*!
 * \brief Prints the help text to the console
 * \param io I/O channel to be used
 */
static void TACHO_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*)"tacho", (unsigned char*)"Group of tacho commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows tacho help or status\r\n", io->stdOut);
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t TACHO_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"tacho help")==0) {
		TACHO_PrintHelp(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"tacho status")==0) {
		TACHO_PrintStatus(io);
		*handled = TRUE;
	}
	return ERR_OK;
}



#ifdef MASTER_tacho_clshdlr_C_
#undef MASTER_tacho_clshdlr_C_
#endif /* !MASTER_tacho_clshdlr_C_ */
