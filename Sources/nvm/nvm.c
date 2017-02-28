#define MASTER_NVM_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "nvm_cfg.h"
#include "nvm.h"
#include "nvm_Types.h"
#include "Platform.h"

/*======================================= >> #DEFINES << =========================================*/




/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void InitNVMCValues(void);



/*=================================== >> GLOBAL VARIABLES << =====================================*/




/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void InitNVMCValues(void) {

}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
 void NVM_Init(void){
	 InitNVMCValues();
 }



#ifdef MASTER_NVM_C_
#undef MASTER_NVM_C_
#endif
