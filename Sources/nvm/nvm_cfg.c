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
#define MASTER_NVM_CFG_C_

#include "Platform.h"
#include "nvm_cfg.h"
#include "IFsh1.h"

static bool isErased(uint8_t *ptr, int nofBytes) {
	while (nofBytes>0) {
		if (*ptr!=0xFF) {
			return FALSE; /* byte not erased */
		}
		ptr++;
		nofBytes--;
	}
	return TRUE;
}

uint8_t NVMC_SaveDeviceAddress16(void *data, size_t dataSize) {
	uint8_t res;

	if (dataSize!=2) {
		return ERR_OVERFLOW;
	}
	res = IFsh1_SetBlockFlash((byte*)data, (IFsh1_TAddress)(NVMC_GetDeviceAddr16Addr()), dataSize);
	return res;
}


uint8_t NVMC_SaveReflectanceData(void *data, uint16_t dataSize) {
	if (dataSize>NVMC_REFLECTANCE_DATA_SIZE) {
		return ERR_OVERFLOW;
	}
	return IFsh1_SetBlockFlash(data, (IFsh1_TAddress)(NVMC_REFLECTANCE_DATA_START_ADDR), dataSize);
}

void *NVMC_GetReflectanceData(void) {
	if (isErased((uint8_t*)NVMC_REFLECTANCE_DATA_START_ADDR, NVMC_REFLECTANCE_DATA_SIZE)) {
		return NULL;
	}
	return (void*)NVMC_REFLECTANCE_DATA_START_ADDR;
}

uint8_t NVMC_SaveRobotData(NVMC_RobotData *data) {
	return IFsh1_SetBlockFlash((void*)data, (IFsh1_TAddress)(NVMC_ROBOT_DATA_START_ADDR), sizeof(NVMC_RobotData));
}

const NVMC_RobotData *NVMC_GetRobotData(void) {
	if (isErased((uint8_t*)NVMC_ROBOT_DATA_START_ADDR, sizeof(NVMC_RobotData))) {
		return NULL;
	}
	return (void*)NVMC_ROBOT_DATA_START_ADDR;
}


#ifdef MASTER_NVM_CFG_C_
#undef MASTER_NVM_CFG_C_
#endif
