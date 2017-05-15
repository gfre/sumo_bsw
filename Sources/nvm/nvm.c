/***********************************************************************************************//**
 * @file		nvm.c
 * @ingroup		nvm
 * @brief 		Implementation of the Non-Volatile-Memory (NVM) storage.
 *
 * This software component provides an implementation to store and retrieve data and parameter values
 * from the on-chip memory of the micro-controller MK22FX512VLK12. In this project the enitre 128KB
 * FlexNVM is used as NVM and no EEPROM is emulated.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_NVM_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "nvm_cfg.h"
#include "nvm.h"
#include "nvm_api.h"
#include "Platform.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef IFsh1_TDataAddress NVM_DataAddr_t;


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static bool isErased(uint8_t *addr_, uint16_t byteCount_);
static inline StdRtn_t SaveBlock2NVM(const NVM_DataAddr_t data_, const NVM_Addr_t nvmAddr_, const uint16 byteCount_, const uint16 xptdByteCount_);
static inline StdRtn_t ReadBlockFromNVM(NVM_DataAddr_t data_, const NVM_Addr_t nvmAddr_, const uint16_t byteCount_);
static void NVM_InitValues(void);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
const NVM_RomCfg_t *romCfg = NULL;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static bool isErased(uint8_t *addr_, uint16_t byteCout_) {
	while (byteCout_>0) {
		if (*addr_!=0xFF) {
			return FALSE; /* byte not erased */
		}
		addr_++;
		byteCout_--;
	}
	return TRUE;
}


static inline StdRtn_t SaveBlock2NVM(const NVM_DataAddr_t data_, const NVM_Addr_t nvmAddr_, const uint16 byteCount_, const uint16 xptdByteCount_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if ( ( Get_NvmDFlashStrtAddr() <= nvmAddr_) && (Get_NvmDFlashEndAddr() - xptdByteCount_) >= nvmAddr_)
	{
		if ( byteCount_ <= xptdByteCount_)
		{
			retVal = (StdRtn_t)IFsh1_SetBlockFlash((IFsh1_TDataAddress)data_, (IFsh1_TAddress)nvmAddr_, (word)byteCount_);
		}
		else
		{
			retVal = ERR_OVERFLOW;
		}
	}
	return retVal;
}


static inline StdRtn_t ReadBlockFromNVM(NVM_DataAddr_t data_, const NVM_Addr_t nvmAddr_, const uint16_t byteCount_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if ( ( NULL != data_ ) && ( Get_NvmDFlashStrtAddr() <= nvmAddr_) && (Get_NvmDFlashEndAddr() - byteCount_) >= nvmAddr_)
	{
		if (!isErased((uint8_t*)nvmAddr_, byteCount_))
		{
			retVal = IFsh1_GetBlockFlash((IFsh1_TAddress)nvmAddr_, (IFsh1_TDataAddress)data_, (word)byteCount_);
		}
		else
		{
			retVal = ERR_PARAM_DATA;
		}
	}
	return retVal;
}


static void NVM_InitValues(void)
{
	uint8_t verNVM = 0u;
	uint8_t verROM = 0u;

	StdRtn_t res = NVM_Read_NvmVerFromNVM(&verNVM);
	res |= NVM_Read_NvmVerFromROM(&verROM);


	if( (ERR_OK != res) || (verNVM < verROM) )
	{
		if(ERR_OK != NVM_Restore_AllFromROM())
		{
			/* error handling */
		}
	}
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t NVM_Read_NvmVerFromNVM(uint8_t *nvmVer_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if (NULL != nvmVer_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)nvmVer_, Get_NvmVerStrtAddr(), sizeof(uint8_t));
	}
	return retVal;
}

StdRtn_t NVM_Read_NvmVerFromROM(uint8_t *nvmVer_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != nvmVer_)
	{
		*nvmVer_ = (uint8)romCfg->nvmVer;
		retVal = ERR_OK;
	}
	return  retVal;
}

StdRtn_t NVM_Read_AllFromROM(NVM_RomCfg_t *romCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != romCfg_)
	{
		*romCfg_ = *romCfg;
		retVal = ERR_OK;
	}
	return  retVal;
}


StdRtn_t NVM_Restore_AllFromROM(void)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)&romCfg, Get_BswDFlashStrtAddr(), sizeof(NVM_RomCfg_t),  Get_BswDFlashCfgrdByteCnt());
}

/* PID position control configuration */
StdRtn_t NVM_Save_PIDPosCfg(const NVM_PidCfg_t *posCfg_)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)posCfg_,Get_PidPosCfgStrtAddr(), sizeof(NVM_PidCfg_t),  Get_PidCfgByteCnt());
}

StdRtn_t NVM_Read_PIDPosCfg(NVM_PidCfg_t *posCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != posCfg_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)posCfg_,Get_PidPosCfgStrtAddr(), sizeof(NVM_PidCfg_t));
	}
	return retVal;
}

StdRtn_t NVM_Read_Dflt_PIDPosCfg(NVM_PidCfg_t *posCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != posCfg_)
	{
		*posCfg_ = (NVM_PidCfg_t)romCfg->pidCfgPos;
		retVal = ERR_OK;
	}
	return  retVal;
}




/* PID speed control configuration for the LEFT WHEEL */
StdRtn_t NVM_Save_PIDSpdLeCfg(const NVM_PidCfg_t *spdCfg_)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)spdCfg_,Get_PidSpdLeCfgStrtAddr(), sizeof(NVM_PidCfg_t),  Get_PidCfgByteCnt());
}

StdRtn_t NVM_Read_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if (NULL != spdCfg_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)spdCfg_,Get_PidSpdLeCfgStrtAddr(), sizeof(NVM_PidCfg_t));
	}
	return retVal;
}

StdRtn_t NVM_Read_Dflt_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != spdCfg_)
	{
		*spdCfg_ = (NVM_PidCfg_t)romCfg->pidCfgSpdLe;
		retVal = ERR_OK;
	}
	return  retVal;
}


/* PID speed control configuration for the RIGHT WHEEL */
StdRtn_t NVM_Save_PIDSpdRiCfg(const NVM_PidCfg_t *spdCfg_)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)spdCfg_,Get_PidSpdRiCfgStrtAddr(), sizeof(NVM_PidCfg_t),  Get_PidCfgByteCnt());
}

StdRtn_t NVM_Read_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if (NULL != spdCfg_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)spdCfg_,Get_PidSpdRiCfgStrtAddr(), sizeof(NVM_PidCfg_t));
	}
	return retVal;
}

StdRtn_t NVM_Read_Dflt_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != spdCfg_)
	{
		*spdCfg_ = (NVM_PidCfg_t)romCfg->pidCfgSpdRi;
		retVal = ERR_OK;
	}
	return  retVal;
}



/* Handle ASW storage data with NVM */
StdRtn_t NVM_Save_ASWDataBytesInUnit(const void *pData_, uint8_t unitNum_, uint16_t byteCnt_)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)pData_, (Get_AswDataFlashStrtAddr() + unitNum_*NVM_UNIT_SIZE_ASW) , byteCnt_,  NVM_UNIT_SIZE_ASW);
}

StdRtn_t NVM_Read_ASWDataUnitAddr(void *pDataAddr_, uint8_t unitNum_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if (NULL != pDataAddr_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)pDataAddr_, (Get_AswDataFlashStrtAddr() + unitNum_*NVM_UNIT_SIZE_ASW), NVM_UNIT_SIZE_ASW);
	}
	return retVal;
}


void NVM_Init(void)
{
	romCfg = Get_pRomCfg();
	if( NULL != romCfg )
	{
		NVM_InitValues();
	}
	else
	{
		/* error handling */
	}
}




#ifdef MASTER_NVM_C_
#undef MASTER_NVM_C_
#endif
