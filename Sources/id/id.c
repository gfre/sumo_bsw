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
#include "Platform.h"
#include "KIN1.h"
#include "id.h"
#include "id_cfg.h"

static ID_Sumos currSumo = ID_SUMO_NONE;
static const ID_Cfg_t *idSumoCfg = NULL;

static ID_Sumos IdentifySumo(void) {
	uint8_t res;
	KIN1_UID id;
	ID_Sumos i, sumo;
//	const KIN1_UID *ids = idSumoCfg->ids;

	sumo = ID_SUMO_UNKNOWN;
	res = KIN1_UIDGet(&id);
	if (res==ERR_OK) {
		for(i=(ID_Sumos)0; i<idSumoCfg->idNum && ID_SUMO_UNKNOWN==sumo; i++) {
			if (KIN1_UIDSame(&id, &(idSumoCfg->ids[i]))) {
				sumo = i; /* found it */
			}
		}
	}
	return sumo;
}

ID_Sumos ID_WhichSumo(void) {
	if (currSumo == ID_SUMO_NONE)
		currSumo = IdentifySumo();
	if (currSumo <= ID_SUMO_UNKNOWN)
		return currSumo;
	else
		return ERR_PARAM_ADDRESS;
}

void ID_Deinit(void) {
	currSumo = ID_SUMO_NONE;
}

void ID_Init(void) {
	currSumo = ID_SUMO_NONE;
	idSumoCfg = Get_ID_Cfg();
	//currSumo = IdentifySumo();
}

#ifdef MASTER_ID_C_
#undef MASTER_ID_C_
#endif
