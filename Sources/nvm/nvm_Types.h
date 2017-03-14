/***************************************************************************************************
 * @brief 	Iinterface of the Non-Volatile-Memory (NVM) storage.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This provides an interface of the NVM software component to other software components
 *
 * =================================================================================================
 */

#ifndef NVM_TYPES_H_
#define NVM_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Acon_Types.h"


#ifdef MASTER_nvm_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0xFFu) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct NVM_PidCfg_s
{
	uint16_t pGain100;			/**< proportional gain */
	uint16_t iGain100;			/**< integral gain */
	uint16_t dGain100;			/**< differential gain */
	uint16_t maxSpdPerc;		/**< maximum speed command in percent */
	uint32_t iAntiWindup;		/**< maximum integral value for anti windup procedure */
} NVM_PidCfg_t; /* 12Byte */

typedef struct NVM_RomCfg_s
{
	uint8_t nvmVer;					/**< NVM version						+ 1B mod4 1B */
	uint8_t filler[3];				/**< filler 	 						+ 3B mod4 0B */
	NVM_PidCfg_t pidCfgPos;			/**< PID position control config 	 	+12B mod4 0B */
	NVM_PidCfg_t pidCfgSpdLe;		/**< PID speed control left config 	 	+12B mod4 0B */
	NVM_PidCfg_t pidCfgSpdRi;		/**< PID speed control right config	 	+12B mod4 0B */
} NVM_RomCfg_t; /* 1 + 3 + 3*12 = 40 Byte*/

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Function reads the NVM version number
 * @param nvmVer_ NVM version (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t NVM_Read_NvmVerFromNVM(uint8_t *nvmVer_);

/**
 * @brief Function reads the ROM version number
 * @param nvmVer_ ROM version (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t NVM_Read_NvmVerFromROM(uint8_t *nvmVer_);

/**
 * @brief Function reads all default parameters from  ROM saved with const qualifier in programm flash
 * @param romCfg_ default parameters (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t NVM_Read_AllFromROM(NVM_RomCfg_t *romCfg_);

/**
 * @brief Function restores NVM with all parameters from ROM
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Restore_AllFromROM(void);

/**
 * @brief Function saves PID parameters of position control to NVM
 * @param posCfg_ position control configuration
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Save_PIDPosCfg(const NVM_PidCfg_t *posCfg_);

/**
 * @brief Function reads PID parameters of position control from NVM
 * @param posCfg_ position control configuration (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Read_PIDPosCfg(NVM_PidCfg_t *posCfg_);

/**
 * @brief Function reads default PID parameters of position control from ROM
 * @param posCfg_ position control configuration (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Read_Dflt_PIDPosCfg(NVM_PidCfg_t *posCfg_);

/**
 * @brief Function saves PID parameters of left speed control to NVM
 * @param spdCfg_ speed control configuration
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Save_PIDSpdLeCfg(const NVM_PidCfg_t *spdCfg_);

/**
 * @brief Function reads PID parameters of left speed control from NVM
 * @param spdCfg_ speed control configuration (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Read_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_);

/**
 * @brief Function reads default PID parameters of left speed control from ROM
 * @param spdCfg_ speed control configuration (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */

EXTERNAL_ StdRtn_t NVM_Read_Dflt_PIDSpdLeCfg(NVM_PidCfg_t *spdCfg_);

/**
 * @brief Function saves PID parameters of right speed control to NVM
 * @param spdCfg_ speed control configuration
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Save_PIDSpdRiCfg(const NVM_PidCfg_t *spdCfg_);

/**
 * @brief Function reads PID parameters of right speed control from NVM
 * @param spdCfg_ speed control configuration (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Read_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_);

/**
 * @brief Function reads default PID parameters of right speed control from ROM
 * @param spdCfg_ speed control configuration (call by ref)
 * @return Error code, ERR_OK if everything was fine,
 *                     specific ERROR CODE otherwise
 */
EXTERNAL_ StdRtn_t NVM_Read_Dflt_PIDSpdRiCfg(NVM_PidCfg_t *spdCfg_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !NVM_TYPES_H_ */
