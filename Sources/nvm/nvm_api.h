/***********************************************************************************************//**
 * @file		nvm_api.h
 * @ingroup		nvm
 * @brief 		API of the SWC *Non-volatile memory*
 *
 * This API provides a BSW-internal interface of the SWC @ref nvm. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef NVM_API_H_
#define NVM_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Acon_Types.h"
#include "rte_Types.h"

#ifdef MASTER_nvm_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup nvm
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
#if defined( RTE_NVM_UNIT_SIZE_ASW )
	#if RTE_NVM_UNIT_SIZE_ASW
	#define NVM_UNIT_SIZE_ASW		RTE_NVM_UNIT_SIZE_ASW
	#else
	#define NVM_UNIT_SIZE_ASW		(0x40u)
	#undef RTE_NVM_UNIT_SIZE_ASW
	#endif
#else
	#define NVM_UNIT_SIZE_ASW		(0x40u)
#endif



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @typedef NVM_PidCfg_t
 * @brief Data type definition of the structure NVM_PidCfg_s
 *
 * @struct NVM_PidCfg_s
 * @brief This structure defines the parameters of a [PID controller](@ref pid) stored in the NVM.
 */
typedef struct NVM_PidCfg_s
{
	uint16_t pGain100;			/**< proportional gain */
	uint16_t iGain100;			/**< integral gain */
	uint16_t dGain100;			/**< differential gain */
	uint16_t maxSpdPerc;		/**< maximum speed command in percent */
	uint32_t iAntiWindup;		/**< maximum integral value for anti windup procedure */
} NVM_PidCfg_t; /* 12Byte */

/**
 * @typedef NVM_RomCfg_t
 * @brief Data type definition of the structure NVM_RomCfg_s
 *
 * @struct NVM_RomCfg_s
 * @brief This structure defines a data configuration which mirrors all parameters stored in the NVM.
 * It is used to store constant default values of all parameters in a ROM-similar fashion.
 */
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

/**
 * @brief This function saves a number of data bytes within a certain ASW data into the NVM
 * @param pData_ reference to the ASW data unit
 * @param unitNum_ number of the ASW data unit where the data shall be saved to
 * @param byteCnt_ count of bytes which shall be saved
 * @return Error code, ERR_OK if everything was fine,
 * 					   ERR_PARAM_OVERFLOW if byteCnt_ exceeds ASW data unit size
 *                     NVM specific ERROR code otherwise
 */
 EXTERNAL_ StdRtn_t NVM_Save_ASWDataBytesInUnit(const void *pData_, uint8_t unitNum_, uint16_t byteCnt_);

 /**
  * @brief This function reads the address of a ASW data unit stored in the NVM
  * @param pDataAddr_ reference to the start address of the ASW data unit
  * @param unitNum_ number of the ASW data unit which shall be read
  * @return Error code, ERR_OK if everything was fine,
  * 					ERR_PARAM_DATA if data was erased,
  *                     ERR_PARAM_ADDRESS if data address is invalid,
  *                     NVM specific ERROR code otherwise
  */
 EXTERNAL_ StdRtn_t NVM_Read_ASWDataUnitAddr(void *pDataAddr_, uint8_t unitNum_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_API_H_ */
