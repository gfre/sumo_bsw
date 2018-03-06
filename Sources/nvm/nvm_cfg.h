/***********************************************************************************************//**
 * @file		nvm_cfg.h
 * @ingroup		nvm
 * @brief 		SWC-internal configuration interface of the SWC @a NVM
 *
 * This header file provides an internal interface within the software component SWC @ref nvm
 * for the configuration of the data storage within the internal non-volatile memory
 * of the MCU.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @note Interface for SWC-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef NVM_CFG_H_
#define NVM_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "nvm_api.h"
#include "IFsh1.h"


#ifdef MASTER_NVM_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup nvm
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @brief Data type (re-)definition for an address parameter of FLASH memory
 */
typedef IFsh1_TAddress NVM_Addr_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This function returns the start address of the entire NVM data flash memory
 * @return data flash address
 */
EXTERNAL_ const NVM_Addr_t Get_NvmDFlashStrtAddr(void);

/**
 * @brief This function returns the end address of the entire NVM data flash memory
 * @return data flash address
 */
EXTERNAL_ const NVM_Addr_t Get_NvmDFlashEndAddr(void);

/**
 * @brief This function returns the start address of the NVM section for the basic software
 * @return data flash address
 */
EXTERNAL_ const NVM_Addr_t Get_BswDFlashStrtAddr(void);

/**
 * @brief This function returns the currently configured byte count of NVM data for the basic software
 * @return byte count
 */
EXTERNAL_ const uint8_t Get_BswDFlashCfgrdByteCnt(void);

/**
 * @brief This function returns the start address of the NVM section for the application software
 * @return data flash address
 */
EXTERNAL_ const NVM_Addr_t Get_AswDataFlashStrtAddr(void);

/**
 * @brief This function returns the reference to the configured ROM data. It is implemented as
 * static RAM variables
 * @return reference to the ROM configuration data
 */
EXTERNAL_ const NVM_RomCfg_t *Get_pRomCfg(void);

/**
 * @brief This function returns the NVM start address of the NVM version data variable
 * @return data flash address
 */

EXTERNAL_ const NVM_Addr_t Get_NvmVerStrtAddr(void);

/**
 * @brief This function returns the NVM start address of the configuration data
 * for the PID position controller
 * @return data flash address
 */
EXTERNAL_ const NVM_Addr_t Get_PidPosCfgStrtAddr(void);

/**
 * @brief This function returns the NVM start address of the configuration data
 * for the PID speed controller of the wheel on the left-hand side
 * @return data flash address
 */
EXTERNAL_ const NVM_Addr_t Get_PidSpdLeCfgStrtAddr(void);

/**
 * @brief This function returns the NVM start address of the configuration data
 * for the PID speed controller of the wheel on the right-hand side
 * @return data flash address
 */
EXTERNAL_ const NVM_Addr_t Get_PidSpdRiCfgStrtAddr(void);

/**
 * @brief This function returns the byte count of NVM data for PID controller configuration
 * @return byte count
 */
EXTERNAL_ const uint8_t Get_PidCfgByteCnt(void);

/**
 * @brief This function returns the NVM start address of the calibration data
 * for the reflectance sensors.
 * @return data flash address
 */
EXTERNAL_ const NVM_Addr_t Get_ReflCalibDataStrtAddr(void);
/**
 * @brief This function returns the byte count of calibration data of the reflectance sensors
 * @return byte count
 */
EXTERNAL_ const uint8_t Get_ReflCalibDataByteCnt(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_CFG_H_ */
