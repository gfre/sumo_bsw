/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	23.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file nvm_cls.c
 * 
 *==================================================================================================
 */

#define MASTER_nvm_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "nvm_clshdlr.h"
#include "nvm_Types.h"


/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t NVM_PrintHelp(const CLS1_StdIOType *io);
static uint8_t NVM_PrintStatus(const CLS1_StdIOType *io);
static uint8_t NVM_PrintVersion(const CLS1_StdIOType *io);


/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t NVM_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*)"nvm", (unsigned char*)"Group of NVM commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows NVM help or status\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  version", (unsigned char*)"Shows NVM version\r\n", io->stdOut);
	return ERR_OK;
}

static uint8_t NVM_PrintStatus(const CLS1_StdIOType *io) {
	CLS1_SendStatusStr((unsigned char*)"nvm", (unsigned char*)"\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  status", "no status information available", io->stdOut);
	return ERR_OK;
}


static uint8_t NVM_PrintVersion(const CLS1_StdIOType *io)
{
	uint8_t ver = 0xFFu;
	ver = NVM_Get_NvmVer();
	CLS1_SendStatusStr((unsigned char*)"  version", "", io->stdOut);
	CLS1_SendNum8u(ver,io->stdOut);
	CLS1_SendStr("\r\n", io->stdOut);
	return ERR_OK;
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t NVM_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
 	uint8_t res = ERR_OK;
 	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"nvm help")==0) {
 		*handled = TRUE;
 		return NVM_PrintHelp(io);
 	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"nvm status")==0) {
 		*handled = TRUE;
 		return NVM_PrintStatus(io);
 	} else if (UTIL1_strcmp((char*)cmd, (char*)"nvm version")==0) {
 		*handled = TRUE;
 		return NVM_PrintVersion(io);
 	}
 	return res;
 }



#ifdef MASTER_nvm_clshdlr_C_
#undef MASTER_nvm_clshdlr_C_
#endif /* !MASTER_nvm_clshdlr_C_ */
