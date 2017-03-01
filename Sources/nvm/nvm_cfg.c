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
#define NVM_DFLASH_START_ADDR          		(IntFlashLdd1_DFLASH_ADDRESS) 					/* 0x10000000LU DFLASH, NVRM_Config, start address of configuration data in flash */
#define NVM_DFLASH_BLOCK_SIZE          		(IntFlashLdd1_DFLASH_SIZE)						/* 0x00020000LU */
#define NVM_DFLASH_ERASABLE_UNIT_SIZE  		(IntFlashLdd1_DFLASH_ERASABLE_UNIT_SIZE)		/* 0x1000LU */
#define NVM_DFLASH_PROT_UNIT_SIZE      		(IntFlashLdd1_DFLASH_PROT_UNIT_SIZE)			/* 0x4000LU */

#define NVM_ERASABLE_UNIT_SIZE				(IntFlashLdd1_ERASABLE_UNIT_SIZE)       		/* 0x1000LU */
#define NVM_ERASABLE_UNIT_MASK				(IntFlashLdd1_ERASABLE_UNIT_MASK)       		/* 0x0FFFLU */
#define NVM_WRITE_UNIT_SIZE					(IntFlashLdd1_WRITE_UNIT_SIZE)          		/* 0x08LU   */
#define NVM_WRITE_UNIT_MASK					(IntFlashLdd1_WRITE_UNIT_MASK)         			/* 0x07LU   */



 /* NVM Version number  */
#define NVM_VERSION (0x01u)

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

#define PID_P_GAIN_POS_START_ADDR				(NVM_DFLASH_START_ADDR)
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

/*=================================== >> TYPE DEFINITIONS << =====================================*/


typedef struct NVM_RomCfg_s
{
	NVM_PidCfg_t pidCfgPos;
	NVM_PidCfg_t pidCfgSpdLe;
	NVM_PidCfg_t pidCfgSpdRi;
} NVM_RomCfg_t;




/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static bool isErased(uint8_t *addr_, uint16_t byteCount_);
static inline StdRtn_t SaveBlock2Flash(const void *data_, const IFsh1_TAddress flashAddr_, const uint16 byteCount_, const uint16 xptdByteCount_);
static inline StdRtn_t ReadBlockFromFlash(void **data_, const IFsh1_TAddress flashAddr_, const uint16 byteCount_);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
const static NVM_RomCfg_t romCfg =
{
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


static inline StdRtn_t SaveBlock2Flash(const void *data_, const IFsh1_TAddress flashAddr_, const uint16 byteCount_, const uint16 xptdByteCount_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if ( ( NULL != data_ ) && ( NULL != (void *)flashAddr_) )
	{
		if ( byteCount_ <= xptdByteCount_)
		{
			retVal = (StdRtn_t)IFsh1_SetBlockFlash((IFsh1_TDataAddress)data_, flashAddr_, byteCount_);
		}
		else
		{
			retVal = ERR_OVERFLOW;
		}
	}
	return retVal;
}


static inline StdRtn_t ReadBlockFromFlash(void **data_, const IFsh1_TAddress flashAddr_, const uint16 byteCount_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if (NULL != data_ )
	{
		if (!isErased((uint8_t*)flashAddr_, byteCount_))
		{
			 *data_ = (void *)flashAddr_;
			 retVal = ERR_OK;
		}
		else
		{
			*data_ = NULL;
			retVal = ERR_PARAM_DATA;
		}
	}
	return retVal;
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t NVM_Get_NvmVer(void) 							      {return (uint8_t )NVM_VERSION; }

/* PID position control configuration */
StdRtn_t NVM_Save_PIDpGainPos(const uint16_t pGain_)
{
	return SaveBlock2Flash((const void *)&pGain_,PID_P_GAIN_POS_START_ADDR, sizeof(uint16_t),  PID_P_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDiGainPos(const uint16_t iGain_)
{
	return SaveBlock2Flash((const void *)&iGain_,PID_I_GAIN_POS_START_ADDR, sizeof(uint16_t),  PID_I_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDdGainPos(const uint16_t dGain_)
{
	return SaveBlock2Flash((const void *)&dGain_,PID_D_GAIN_POS_START_ADDR, sizeof(uint16_t),  PID_D_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDMaxSpdPercPos(const uint16_t maxSpdPerc_)
{
	return SaveBlock2Flash((const void *)&maxSpdPerc_,PID_MAX_SPEED_PERC_POS_START_ADDR, sizeof(uint16_t),  PID_MAX_SPEED_PERC_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDiAntiWindUpPos(const uint32_t iAntiWindUp_)
{
	return SaveBlock2Flash((const void *)&iAntiWindUp_,PID_I_ANTIWINDUP_POS_START_ADDR, sizeof(uint32_t),  PID_I_ANTIWINDUP_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDPosCfg(const NVM_PidCfg_t *posCfg_)
{
	return SaveBlock2Flash((const void *)posCfg_,PID_POS_CFG_START_ADDR, sizeof(NVM_PidCfg_t),  PID_CFG_BYTE_COUNT);
}

StdRtn_t NVM_Read_PIDPosCfg(NVM_PidCfg_t *posCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t *addr = NULL;

	if (NULL != posCfg_)
	{
		retVal = ReadBlockFromFlash((void *)&addr,PID_POS_CFG_START_ADDR, sizeof(NVM_PidCfg_t));
		*posCfg_ = *addr;
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
StdRtn_t NVM_Save_PIDpGainSpdLe(const uint16_t pGain_)
{
	return SaveBlock2Flash((const void *)&pGain_,PID_P_GAIN_SPDLE_START_ADDR, sizeof(uint16_t),  PID_P_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDiGainSpdLe(const uint16_t iGain_)
{
	return SaveBlock2Flash((const void *)&iGain_,PID_I_GAIN_SPDLE_START_ADDR, sizeof(uint16_t),  PID_I_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDdGainSpdLe(const uint16_t dGain_)
{
	return SaveBlock2Flash((const void *)&dGain_,PID_D_GAIN_SPDLE_START_ADDR, sizeof(uint16_t),  PID_D_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDMaxSpdPercSpdLe(const uint16_t maxSpdPerc_)
{
	return SaveBlock2Flash((const void *)&maxSpdPerc_,PID_MAX_SPEED_PERC_SPDLE_START_ADDR, sizeof(uint16_t),  PID_MAX_SPEED_PERC_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDiAntiWindUpSpdLe(const uint32_t iAntiWindUp_)
{
	return SaveBlock2Flash((const void *)&iAntiWindUp_,PID_I_ANTIWINDUP_SPDLE_START_ADDR, sizeof(uint32_t),  PID_I_ANTIWINDUP_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDSpdLeCfg(const NVM_PidCfg_t *spdCfg_)
{
	return SaveBlock2Flash((const void *)spdCfg_,PID_SPDLE_CFG_START_ADDR, sizeof(NVM_PidCfg_t),  PID_CFG_BYTE_COUNT);
}

StdRtn_t NVM_Read_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t *addr = NULL;

	if (NULL != spdCfg_)
	{
		retVal = ReadBlockFromFlash((void *)&addr,PID_SPDLE_CFG_START_ADDR, sizeof(NVM_PidCfg_t));
		*spdCfg_ = *addr;
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
StdRtn_t NVM_Save_PIDpGainSpdRi(const uint16_t pGain_)
{
	return SaveBlock2Flash((const void *)&pGain_,PID_P_GAIN_SPDRI_START_ADDR, sizeof(uint16_t),  PID_P_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDiGainSpdRi(const uint16_t iGain_)
{
	return SaveBlock2Flash((const void *)&iGain_,PID_I_GAIN_SPDRI_START_ADDR, sizeof(uint16_t),  PID_I_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDdGainSpdRi(const uint16_t dGain_)
{
	return SaveBlock2Flash((const void *)&dGain_,PID_D_GAIN_SPDRI_START_ADDR, sizeof(uint16_t),  PID_D_GAIN_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDMaxSpdPercSpdRi(const uint16_t maxSpdPerc_)
{
	return SaveBlock2Flash((const void *)&maxSpdPerc_,PID_MAX_SPEED_PERC_SPDRI_START_ADDR, sizeof(uint16_t),  PID_MAX_SPEED_PERC_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDiAntiWindUpSpdRi(const uint32_t iAntiWindUp_)
{
	return SaveBlock2Flash((const void *)&iAntiWindUp_,PID_I_ANTIWINDUP_SPDRI_START_ADDR, sizeof(uint32_t),  PID_I_ANTIWINDUP_BYTE_COUNT);
}

StdRtn_t NVM_Save_PIDSpdRiCfg(const NVM_PidCfg_t *spdCfg_)
{
	return SaveBlock2Flash((const void *)spdCfg_,PID_SPDRI_CFG_START_ADDR, sizeof(NVM_PidCfg_t),  PID_CFG_BYTE_COUNT);
}

StdRtn_t NVM_Read_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	NVM_PidCfg_t *addr = NULL;

	if (NULL != spdCfg_)
	{
		retVal = ReadBlockFromFlash((void *)&addr,PID_SPDRI_CFG_START_ADDR, sizeof(NVM_PidCfg_t));
		*spdCfg_ = *addr;
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
