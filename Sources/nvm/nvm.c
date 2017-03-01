/***************************************************************************************************
 * @brief 	Implementation of the Non-Volatile-Memory (NVM) storage.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This provides an implementation to store and retrieve data from the on-chip memory.
 *
 * =================================================================================================
 */

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
