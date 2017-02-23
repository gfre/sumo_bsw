#define MASTER_NVM_C_
#include "nvm_cfg.h"
#include "nvm.h"

#define NVMC_VERSION  0x03

static const NVMC_RobotData *RoboDataPtr;

static void InitNVMCValues(void);


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



#ifdef MASTER_NVM_C_
#undef MASTER_NVM_C_
#endif
