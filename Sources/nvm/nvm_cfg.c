/***************************************************************************************************
 * @brief 	Configuration of the Non-Volatile-Memory (NVM) storage.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This provides the configuration to store and retrieve data from the on-chip memory.
 *
 * =================================================================================================
 */

#define MASTER_NVM_CFG_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "nvm_cfg.h"
#include "IFsh1.h"
#include "nvm_Types.h"
#include "Pid.h"



/*======================================= >> #DEFINES << =========================================*/
 /*
  * NVM Version number
  */
#define NVM_VERSION (0x01u)
/* ========================= */

#define NVM_DFLASH_START_ADDR          		(IntFlashLdd1_DFLASH_ADDRESS) 					/* 0x10000000LU DFLASH, NVRM_Config, start address of configuration data in flash */
#define NVM_DFLASH_BLOCK_SIZE          		(IntFlashLdd1_DFLASH_SIZE)						/* 0x00020000LU */
#define NVM_DFLASH_ERASABLE_UNIT_SIZE  		(IntFlashLdd1_DFLASH_ERASABLE_UNIT_SIZE)		/* 0x1000LU */
#define NVM_DFLASH_PROT_UNIT_SIZE      		(IntFlashLdd1_DFLASH_PROT_UNIT_SIZE)			/* 0x4000LU */

#define NVM_ERASABLE_UNIT_SIZE				(IntFlashLdd1_ERASABLE_UNIT_SIZE)       		/* 0x1000LU */
#define NVM_ERASABLE_UNIT_MASK				(IntFlashLdd1_ERASABLE_UNIT_MASK)       		/* 0x0FFFLU */
#define NVM_WRITE_UNIT_SIZE					(IntFlashLdd1_WRITE_UNIT_SIZE)          		/* 0x08LU   */
#define NVM_WRITE_UNIT_MASK					(IntFlashLdd1_WRITE_UNIT_MASK)         			/* 0x07LU   */

#define BYTE_FILLER( numOfBytes_ )					(numOfBytes_)

/* Define byte counts */
#define PID_P_GAIN_BYTE_COUNT 						(sizeof(uint16_t))
#define PID_I_GAIN_BYTE_COUNT 						(sizeof(uint16_t))
#define PID_D_GAIN_BYTE_COUNT 						(sizeof(uint16_t))
#define PID_MAX_SPEED_PERC_BYTE_COUNT				(sizeof(uint16_t))
#define PID_I_ANTIWINDUP_BYTE_COUNT					(sizeof(uint32_t))
#define PID_CFG_BYTE_COUNT 							(PID_P_GAIN_BYTE_COUNT + PID_I_GAIN_BYTE_COUNT + PID_D_GAIN_BYTE_COUNT \
													+ PID_MAX_SPEED_PERC_BYTE_COUNT + PID_I_ANTIWINDUP_BYTE_COUNT)

/* Define default values */
#define PID_P_GAIN_POS_DEFAULT				(1000u)
#define PID_I_GAIN_POS_DEFAULT				(1u)
#define PID_D_GAIN_POS_DEFAULT				(50u)
#define PID_I_ANTIWINDUP_POS_DEFAULT		(200u)

#define PID_P_GAIN_SPD_DEFAULT				(2000u)
#define PID_I_GAIN_SPD_DEFAULT				(80u)
#define PID_D_GAIN_SPD_DEFAULT				(0u)
#define PID_I_ANTIWINDUP_SPD_DEFAULT		(120000u)

#define PID_MAX_SPEED_PERC_DEFAULT			(100u)



 /*  Define the memory areas
  * =========================
  */

#define NVM_VERSION_START_ADDR					(NVM_DFLASH_START_ADDR)
#define NVM_VERSION_BYTE_COUNT					(sizeof(uint8))
#define NVM_VERSION_END_ADDR					(NVM_VERSION_START_ADDR + NVM_VERSION_BYTE_COUNT + BYTE_FILLER(3u) )

#define PID_P_GAIN_POS_START_ADDR				(NVM_VERSION_END_ADDR)
#define PID_P_GAIN_POS_END_ADDR					(PID_P_GAIN_POS_START_ADDR + PID_P_GAIN_BYTE_COUNT)

#define PID_I_GAIN_POS_START_ADDR				(PID_P_GAIN_POS_END_ADDR)
#define PID_I_GAIN_POS_END_ADDR					(PID_I_GAIN_POS_START_ADDR + PID_I_GAIN_BYTE_COUNT)

#define PID_D_GAIN_POS_START_ADDR				(PID_I_GAIN_POS_END_ADDR)
#define PID_D_GAIN_POS_END_ADDR					(PID_D_GAIN_POS_START_ADDR + PID_D_GAIN_BYTE_COUNT)

#define PID_MAX_SPEED_PERC_POS_START_ADDR		(PID_D_GAIN_POS_END_ADDR)
#define PID_MAX_SPEED_PERC_POS_END_ADDR			(PID_MAX_SPEED_PERC_POS_START_ADDR + PID_MAX_SPEED_PERC_BYTE_COUNT)

#define PID_I_ANTIWINDUP_POS_START_ADDR			(PID_MAX_SPEED_PERC_POS_END_ADDR)
#define PID_I_ANTIWINDUP_POS_END_ADDR			(PID_I_ANTIWINDUP_POS_START_ADDR + PID_I_ANTIWINDUP_BYTE_COUNT)

#define PID_POS_CFG_START_ADDR					(PID_P_GAIN_POS_START_ADDR)
#define PID_POS_CFG_END_ADDR					(PID_POS_CFG_START_ADDR + PID_CFG_BYTE_COUNT)


#define PID_P_GAIN_SPDLE_START_ADDR				(PID_I_ANTIWINDUP_POS_END_ADDR)
#define PID_P_GAIN_SPDLE_END_ADDR				(PID_P_GAIN_SPDLE_START_ADDR + PID_P_GAIN_BYTE_COUNT)

#define PID_I_GAIN_SPDLE_START_ADDR				(PID_P_GAIN_SPDLE_END_ADDR)
#define PID_I_GAIN_SPDLE_END_ADDR				(PID_I_GAIN_SPDLE_START_ADDR + PID_I_GAIN_BYTE_COUNT)

#define PID_D_GAIN_SPDLE_START_ADDR				(PID_I_GAIN_SPDLE_END_ADDR)
#define PID_D_GAIN_SPDLE_END_ADDR				(PID_D_GAIN_SPDLE_START_ADDR + PID_D_GAIN_BYTE_COUNT)

#define PID_MAX_SPEED_PERC_SPDLE_START_ADDR		(PID_D_GAIN_SPDLE_END_ADDR)
#define PID_MAX_SPEED_PERC_SPDLE_END_ADDR		(PID_MAX_SPEED_PERC_SPDLE_START_ADDR + PID_MAX_SPEED_PERC_BYTE_COUNT)

#define PID_I_ANTIWINDUP_SPDLE_START_ADDR		(PID_MAX_SPEED_PERC_SPDLE_END_ADDR)
#define PID_I_ANTIWINDUP_SPDLE_END_ADDR			(PID_I_ANTIWINDUP_SPDLE_START_ADDR + PID_I_ANTIWINDUP_BYTE_COUNT)

#define PID_SPDLE_CFG_START_ADDR				(PID_P_GAIN_SPDLE_START_ADDR)
#define PID_SPDLE_CFG_END_ADDR					(PID_SPDLE_CFG_START_ADDR + PID_CFG_BYTE_COUNT)



#define PID_P_GAIN_SPDRI_START_ADDR				(PID_I_ANTIWINDUP_SPDLE_END_ADDR)
#define PID_P_GAIN_SPDRI_END_ADDR				(PID_P_GAIN_SPDRI_START_ADDR + PID_P_GAIN_BYTE_COUNT)

#define PID_I_GAIN_SPDRI_START_ADDR				(PID_P_GAIN_SPDRI_END_ADDR)
#define PID_I_GAIN_SPDRI_END_ADDR				(PID_I_GAIN_SPDRI_START_ADDR + PID_I_GAIN_BYTE_COUNT)

#define PID_D_GAIN_SPDRI_START_ADDR				(PID_I_GAIN_SPDRI_END_ADDR)
#define PID_D_GAIN_SPDRI_END_ADDR				(PID_D_GAIN_SPDRI_START_ADDR + PID_D_GAIN_BYTE_COUNT)

#define PID_MAX_SPEED_PERC_SPDRI_START_ADDR		(PID_D_GAIN_SPDRI_END_ADDR)
#define PID_MAX_SPEED_PERC_SPDRI_END_ADDR		(PID_MAX_SPEED_PERC_SPDRI_START_ADDR + PID_MAX_SPEED_PERC_BYTE_COUNT)

#define PID_I_ANTIWINDUP_SPDRI_START_ADDR		(PID_MAX_SPEED_PERC_SPDRI_END_ADDR)
#define PID_I_ANTIWINDUP_SPDRI_END_ADDR			(PID_I_ANTIWINDUP_SPDRI_START_ADDR + PID_I_ANTIWINDUP_BYTE_COUNT)

#define PID_SPDRI_CFG_START_ADDR				(PID_P_GAIN_SPDRI_START_ADDR)
#define PID_SPDRI_CFG_END_ADDR					(PID_SPDRI_CFG_START_ADDR + PID_CFG_BYTE_COUNT)


#define NVM_DFLASH_CURRENT_END_ADDR				(PID_SPDRI_CFG_END_ADDR)
#define NVM_DFLASH_CURRENT_BYTE_COUNT			(NVM_DFLASH_CURRENT_END_ADDR - NVM_DFLASH_START_ADDR)



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef IFsh1_TDataAddress NVM_DataAddr_t;
typedef IFsh1_TAddress NVM_Addr_t;



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static bool isErased(uint8_t *addr_, uint16_t byteCount_);
static inline StdRtn_t SaveBlock2NVM(const NVM_DataAddr_t data_, const NVM_Addr_t nvmAddr_, const uint16 byteCount_, const uint16 xptdByteCount_);
static inline StdRtn_t ReadBlockFromNVM(NVM_DataAddr_t data_, const NVM_Addr_t nvmAddr_, const uint16_t byteCount_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
const static NVM_RomCfg_t romCfg =
{
		/* NVM Version */ NVM_VERSION,
		/* 3Byte filler*/{0u},
		/* pidCfgPos   */{PID_P_GAIN_POS_DEFAULT, PID_I_GAIN_POS_DEFAULT, PID_D_GAIN_POS_DEFAULT, PID_MAX_SPEED_PERC_DEFAULT, PID_I_ANTIWINDUP_POS_DEFAULT},
		/* pidCfgSpdLe */{PID_P_GAIN_SPD_DEFAULT, PID_I_GAIN_SPD_DEFAULT, PID_D_GAIN_SPD_DEFAULT, PID_MAX_SPEED_PERC_DEFAULT, PID_I_ANTIWINDUP_SPD_DEFAULT},
		/* pidCfgSpdRi */{PID_P_GAIN_SPD_DEFAULT, PID_I_GAIN_SPD_DEFAULT, PID_D_GAIN_SPD_DEFAULT, PID_MAX_SPEED_PERC_DEFAULT, PID_I_ANTIWINDUP_SPD_DEFAULT},
};


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

	if ( ( NULL != data_ ) && ( NVM_DFLASH_START_ADDR <= nvmAddr_) && (NVM_DFLASH_START_ADDR +NVM_DFLASH_BLOCK_SIZE) >= nvmAddr_)
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

	if ( ( NULL != data_ ) && ( NVM_DFLASH_START_ADDR <= nvmAddr_) && (NVM_DFLASH_START_ADDR +NVM_DFLASH_BLOCK_SIZE) >= nvmAddr_)
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



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t NVM_Read_NvmVerFromNVM(uint8_t *nvmVer_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if (NULL != nvmVer_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)nvmVer_, NVM_VERSION_START_ADDR, sizeof(uint8_t));
	}
	return retVal;
}

StdRtn_t NVM_Read_NvmVerFromROM(uint8_t *nvmVer_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != nvmVer_)
	{
		*nvmVer_ = (uint8)romCfg.nvmVer;
		retVal = ERR_OK;
	}
	return  retVal;
}

StdRtn_t NVM_Read_AllFromROM(NVM_RomCfg_t *romCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != romCfg_)
	{
		*romCfg_ = romCfg;
		retVal = ERR_OK;
	}
	return  retVal;
}


StdRtn_t NVM_Restore_AllFromROM(void)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)&romCfg, NVM_DFLASH_START_ADDR, sizeof(NVM_RomCfg_t),  NVM_DFLASH_CURRENT_BYTE_COUNT);
}

/* PID position control configuration */
StdRtn_t NVM_Save_PIDPosCfg(const NVM_PidCfg_t *posCfg_)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)posCfg_,PID_POS_CFG_START_ADDR, sizeof(NVM_PidCfg_t),  PID_CFG_BYTE_COUNT);
}

StdRtn_t NVM_Read_PIDPosCfg(NVM_PidCfg_t *posCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != posCfg_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)posCfg_,PID_POS_CFG_START_ADDR, sizeof(NVM_PidCfg_t));
	}
	return retVal;
}

StdRtn_t NVM_Read_Dflt_PIDPosCfg(NVM_PidCfg_t *posCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != posCfg_)
	{
		*posCfg_ = (NVM_PidCfg_t)romCfg.pidCfgPos;
		retVal = ERR_OK;
	}
	return  retVal;
}




/* PID speed control configuration for the LEFT WHEEL */
StdRtn_t NVM_Save_PIDSpdLeCfg(const NVM_PidCfg_t *spdCfg_)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)spdCfg_,PID_SPDLE_CFG_START_ADDR, sizeof(NVM_PidCfg_t),  PID_CFG_BYTE_COUNT);
}

StdRtn_t NVM_Read_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if (NULL != spdCfg_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)spdCfg_,PID_SPDLE_CFG_START_ADDR, sizeof(NVM_PidCfg_t));
	}
	return retVal;
}

StdRtn_t NVM_Read_Dflt_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != spdCfg_)
	{
		*spdCfg_ = (NVM_PidCfg_t)romCfg.pidCfgSpdLe;
		retVal = ERR_OK;
	}
	return  retVal;
}


/* PID speed control configuration for the RIGHT WHEEL */
StdRtn_t NVM_Save_PIDSpdRiCfg(const NVM_PidCfg_t *spdCfg_)
{
	return SaveBlock2NVM((const NVM_DataAddr_t)spdCfg_,PID_SPDRI_CFG_START_ADDR, sizeof(NVM_PidCfg_t),  PID_CFG_BYTE_COUNT);
}

StdRtn_t NVM_Read_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if (NULL != spdCfg_)
	{
		retVal = ReadBlockFromNVM((NVM_DataAddr_t)spdCfg_,PID_SPDRI_CFG_START_ADDR, sizeof(NVM_PidCfg_t));
	}
	return retVal;
}

StdRtn_t NVM_Read_Dflt_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != spdCfg_)
	{
		*spdCfg_ = (NVM_PidCfg_t)romCfg.pidCfgSpdRi;
		retVal = ERR_OK;
	}
	return  retVal;
}

#ifdef MASTER_NVM_CFG_C_
#undef MASTER_NVM_CFG_C_
#endif
