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
#include "nvm_api.h"
#include "Platform.h"

/*======================================= >> #DEFINES << =========================================*/




/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void NVM_InitValues(void);



/*=================================== >> GLOBAL VARIABLES << =====================================*/




/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void NVM_InitValues(void)
{
	uint8_t verNVM = 0u;
	uint8_t verROM = 0u;
	NVM_RomCfg_t romCfg = {0u};

	StdRtn_t res = NVM_Read_NvmVerFromNVM(&verNVM);
	res |= NVM_Read_NvmVerFromROM(&verROM);


	if( (ERR_OK != res) || (verNVM < verROM) )
	{
		if(ERR_OK != NVM_Restore_AllFromROM())
		{
			/* TODO - error handling */
		}
	}
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void NVM_Init(void)
{
	NVM_InitValues();
}




#ifdef MASTER_NVM_C_
#undef MASTER_NVM_C_
#endif
