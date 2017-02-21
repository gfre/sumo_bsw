#define MASTER_NVM_C_
#include "CLS1.h"
#include "nvm_cfg.h"
#include "nvm.h"

#define NVMC_VERSION  0x03

static const NVMC_RobotData *RoboDataPtr;

static uint8_t NVM_PrintHelp(const CLS1_StdIOType *io);
static uint8_t NVM_PrintStatus(const CLS1_StdIOType *io);
static void InitNVMCValues(void);


static uint8_t NVM_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*)"nvm", (unsigned char*)"Group of nvm commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows nvm help or status\r\n", io->stdOut);
	return ERR_OK;
}

static uint8_t NVM_PrintStatus(const CLS1_StdIOType *io) {
	CLS1_SendStatusStr((unsigned char*)"nvm", (unsigned char*)"\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  status", "no status information available", io->stdOut);
	return ERR_OK;
}


static void InitNVMCValues(void) {
	const NVMC_RobotData *ptr;
	NVMC_RobotData data;
	uint8_t res;


	ptr = NVMC_GetRobotData();
	if (ptr==NULL || ptr->version != NVMC_VERSION) {
		data.version = NVMC_VERSION;
		res = NVMC_SaveRobotData(&data);
		if (res!=ERR_OK) {
			for(;;); /* error */
		}
	}
}


 void NVM_Init(void){
	 InitNVMCValues();
 }


uint8_t NVM_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
 	uint8_t res = ERR_OK;
 	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"nvm help")==0) {
 		*handled = TRUE;
 		return NVM_PrintHelp(io);
 	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"nvm status")==0) {
 		*handled = TRUE;
 		return NVM_PrintStatus(io);
 	}
 	return res;
 }
#ifdef MASTER_NVM_C_
#undef MASTER_NVM_C_
#endif
