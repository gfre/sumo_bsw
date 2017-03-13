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
static void NVM_InitValues(void);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
/* TODO
 * - remove single save fcts. only PID blocks need to be saved
 * - autosave when PID gains are updated, no explicit save needed
 * - restore PID blocks only
 * - typedef void_a void *
 * - typedef NVM_PIDCfg_a NVM_PIDCfg_t *
 * - then change parameters in read functions to pointers to addresses instead of pointers to structs/blocks.
 */




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
		if(ERR_OK != NVM_RestoreAll())
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


StdRtn_t NVM_RestoreAll(void)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_RomCfg_t romCfg = {0u};

 	if( (ERR_OK == NVM_Read_AllFromROM(&romCfg)))
 	{
 		retVal = NVM_Save_All2NVM((const void *)&romCfg);
 	}

 	return retVal;
}


#ifdef MASTER_NVM_C_
#undef MASTER_NVM_C_
#endif
