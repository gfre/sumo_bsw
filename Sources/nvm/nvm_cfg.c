/***********************************************************************************************//**
 * @file		nvm_cfg.c
 * @ingroup		nvm
 * @brief 		Implementation of the configuration of the SWC @a NVM
 *
 * This file implements the configuration of the data storage using the internal non-volatile memory
 * of the MCU for the component SWC @ref nvm and its internal interface.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_NVM_CFG_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "nvm_cfg.h"



/*======================================= >> #DEFINES << =========================================*/
 /*
  * NVM Version number
  */
#define NVM_VERSION (0x01u)
/* ========================= */

#define NVM_DFLASH_START_ADDR          		(IntFlashLdd1_DFLASH_ADDRESS) 					/* 0x10000000LU DFLASH, NVM_Config, start address of configuration data in flash */
#define NVM_DFLASH_BLOCK_SIZE          		(IntFlashLdd1_DFLASH_SIZE)						/* 0x00020000LU */
#define NVM_DFLASH_ERASABLE_UNIT_SIZE  		(IntFlashLdd1_DFLASH_ERASABLE_UNIT_SIZE)		/* 0x1000LU */
#define NVM_DFLASH_PROT_UNIT_SIZE      		(IntFlashLdd1_DFLASH_PROT_UNIT_SIZE)			/* 0x4000LU */

#define NVM_ERASABLE_UNIT_SIZE				(IntFlashLdd1_ERASABLE_UNIT_SIZE)       		/* 0x1000LU */
#define NVM_ERASABLE_UNIT_MASK				(IntFlashLdd1_ERASABLE_UNIT_MASK)       		/* 0x0FFFLU */
#define NVM_WRITE_UNIT_SIZE					(IntFlashLdd1_WRITE_UNIT_SIZE)          		/* 0x08LU   */
#define NVM_WRITE_UNIT_MASK					(IntFlashLdd1_WRITE_UNIT_MASK)         			/* 0x07LU   */

#define BYTE_FILLER( numOfBytes_ )					(numOfBytes_)

/* define BSW and ASW memory blocks */
#define NVM_BSW_DFLASH_START_ADDR			(NVM_DFLASH_START_ADDR)								/* 0x10000000LU */
#define NVM_BSW_DFLASH_SIZE					(NVM_DFLASH_ERASABLE_UNIT_SIZE)						/* 0x00001000LU */
#define NVM_BSW_DFLASH_END_ADDR				(NVM_BSW_DFLASH_START_ADDR + NVM_BSW_DFLASH_SIZE)	/* 0x10001000LU */

#define NVM_ASW_DFLASH_START_ADDR			(NVM_BSW_DFLASH_END_ADDR)							/* 0x10001000LU */
#define NVM_ASW_DFLASH_SIZE					(NVM_DFLASH_ERASABLE_UNIT_SIZE)						/* 0x00001000LU */
#define NVM_ASW_DFLASH_END_ADDR				(NVM_ASW_DFLASH_START_ADDR + NVM_ASW_DFLASH_SIZE)   /* 0x10002000LU */


/* Define byte counts */
#define PID_P_GAIN_BYTE_COUNT 				(sizeof(uint16_t))
#define PID_I_GAIN_BYTE_COUNT 				(sizeof(uint16_t))
#define PID_D_GAIN_BYTE_COUNT 				(sizeof(uint16_t))
#define PID_MAX_SPEED_PERC_BYTE_COUNT		(sizeof(uint16_t))
#define PID_I_ANTIWINDUP_BYTE_COUNT			(sizeof(uint32_t))
#define PID_CFG_BYTE_COUNT 					(PID_P_GAIN_BYTE_COUNT + PID_I_GAIN_BYTE_COUNT + PID_D_GAIN_BYTE_COUNT \
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

#define NVM_VERSION_START_ADDR					NVM_BSW_DFLASH_START_ADDR
#define NVM_VERSION_BYTE_COUNT					(sizeof(uint8_t))
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


#define NVM_BSW_DFLASH_CFGRD_END_ADDR			(PID_SPDRI_CFG_END_ADDR)
#define NVM_BSW_DFLASH_CFGRD_BYTE_COUNT			(NVM_BSW_DFLASH_CFGRD_END_ADDR - NVM_BSW_DFLASH_START_ADDR)



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



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



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
const NVM_Addr_t Get_NvmDFlashStrtAddr(void)		{return NVM_DFLASH_START_ADDR;}
const NVM_Addr_t Get_NvmDFlashEndAddr(void)			{return (NVM_DFLASH_START_ADDR + NVM_DFLASH_BLOCK_SIZE);}

const NVM_Addr_t Get_BswDFlashStrtAddr(void)		{return NVM_BSW_DFLASH_START_ADDR;}
const uint8_t Get_BswDFlashCfgrdByteCnt(void)		{return NVM_BSW_DFLASH_CFGRD_BYTE_COUNT;}

const NVM_Addr_t Get_AswDFlashStrtAddr(void)		{return NVM_ASW_DFLASH_START_ADDR;}

const NVM_Addr_t Get_NvmVerStrtAddr(void)			{return NVM_VERSION_START_ADDR;}

const NVM_Addr_t Get_PidPosCfgStrtAddr(void)		{return PID_POS_CFG_START_ADDR;}
const NVM_Addr_t Get_PidSpdLeCfgStrtAddr(void)		{return PID_SPDLE_CFG_START_ADDR;}
const NVM_Addr_t Get_PidSpdRiCfgStrtAddr(void)		{return PID_SPDRI_CFG_START_ADDR;}
const uint8_t Get_PidCfgByteCnt(void)				{return PID_CFG_BYTE_COUNT;}

const NVM_RomCfg_t *Get_pRomCfg(void)				{return &romCfg;}






#ifdef MASTER_NVM_CFG_C_
#undef MASTER_NVM_CFG_C_
#endif
