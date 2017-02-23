/*******************************************************************************
 * @brief		Module to handle the unique SUMO IDs.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date 		11.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#define MASTER_ID_C_
#include "id.h"
#include "id_Types.h"
#include "id_cfg.h"

static ID_Sumo_t currID = ID_SUMO_NONE;
static const ID_Cfg_t *idCfg = NULL;

static ID_Sumo_t IdentifySumo(void) {
	uint8_t res = 0u;
	KIN1_UID uid = {0u};
	uint8_t index = 0u;
	ID_Sumo_t id = 0;

	id = ID_SUMO_UNKNOWN;
	res = KIN1_UIDGet(&uid);
	if (res==ERR_OK) {
		for(index = 0u; index < idCfg->uidNum && ID_SUMO_UNKNOWN == id; index++) {
			if (KIN1_UIDSame(&uid, &(idCfg->uids[index]))) {
				id = (ID_Sumo_t)index; /* found it */
			}
		}
	}
	return id;
}

ID_Sumo_t Get_SumoID(void) {
	if (ID_SUMO_NONE == currID)
	{
		currID = IdentifySumo();
	}
	if ((ID_SUMO_MAX < currID) || (ID_SUMO_MIN > currID))
	{
		currID = ID_SUMO_UNKNOWN;
	}
	return currID;
}

void ID_Deinit(void) {
	currID = ID_SUMO_NONE;
}

void ID_Init(void) {
	currID = ID_SUMO_NONE;
	idCfg = Get_IDCfg();
}

#ifdef MASTER_ID_C_
#undef MASTER_ID_C_
#endif
