/*******************************************************************************
 * @brief 	Implementation of Non-Volatile-Memory (NVM) storage.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 		10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This provides an implementation to store and retrieve data from the on-chip memory.
 *
 * ==============================================================================
 */

#ifndef NVM_CFG_H_
#define NVM_CFG_H_

#include "Platform.h"

#ifdef MASTER_NVM_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

#define NVMC_FLASH_START_ADDR    0x10000000 /* DFLASH, NVRM_Config, start address of configuration data in flash */
#define NVMC_FLASH_BLOCK_SIZE    0x1000  /* IntFlashLdd1_BLOCK0_ERASABLE_UNIT_SIZE */
#define NVMC_FLASH_ERASED_UINT8  0xFF
#define NVMC_FLASH_ERASED_UINT16 0xFFFF

typedef struct {
	uint8_t version;
} NVMC_RobotData;

/* Define the memory areas */
#define NVMC_DEVICE_ADDR16_START_ADDR     (NVMC_FLASH_START_ADDR)
#define NVMC_DEVICE_ADDR16_DATA_SIZE      (2)
#define NVMC_DEVICE_ADDR16_END_ADDR       (NVMC_DEVICE_ADDR16_START_ADDR+NVMC_DEVICE_ADDR16_DATA_SIZE)

#define NVMC_REFLECTANCE_DATA_START_ADDR  (NVMC_DEVICE_ADDR16_END_ADDR)
#define NVMC_REFLECTANCE_DATA_SIZE        (8*2*2) /* maximum of 8 sensors (min and max) values with 16 bits (2 bytes) */
#define NVMC_REFLECTANCE_END_ADDR         (NVMC_REFLECTANCE_DATA_START_ADDR+NVMC_REFLECTANCE_DATA_SIZE)

#define NVMC_SUMO_DATA_START_ADDR         (NVMC_REFLECTANCE_END_ADDR)
#define NVMC_SUMO_DATA_SIZE               (4) /* 4 bytes of data */
#define NVMC_SUMO_END_ADDR                (NVMC_SUMO_DATA_START_ADDR+NVMC_SUMO_DATA_SIZE)

#define NVMC_ROBOT_DATA_START_ADDR         (NVMC_SUMO_END_ADDR)
#define NVMC_ROBOT_DATA_SIZE               (sizeof(NVMC_RobotData))
#define NVMC_ROBOT_END_ADDR                (NVMC_ROBOT_DATA_START_ADDR+NVMC_ROBOT_DATA_SIZE)

/* Access macros */
EXTERNAL_ uint8_t NVMC_SaveDeviceAddress16(void *data, size_t dataSize);
#define NVMC_GetDeviceAddr16Addr()        ((uint16_t*)(NVMC_DEVICE_ADDR16_START_ADDR))


EXTERNAL_ uint8_t NVMC_SaveReflectanceData(void *data, uint16_t dataSize);
EXTERNAL_ void *NVMC_GetReflectanceData(void);


EXTERNAL_ uint8_t NVMC_SaveRobotData(NVMC_RobotData *data);
EXTERNAL_ const NVMC_RobotData *NVMC_GetRobotData(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_CFG_H_ */
