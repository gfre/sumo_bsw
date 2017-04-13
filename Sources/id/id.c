/***********************************************************************************************//**
 * @file		id.c
 * @ingroup		id
 * @brief 		Implementation of the identification algorithm.
 *
 * This module implements an identification component for a MCU by means of its *Universally
 * Unique Identifier (@b UUID)* provided and enhanced by the firmware component @b KIN1. It
 * maps the UUID to a custom ID according to index of the UUID in a pre-defined table.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.01.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_ID_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "id.h"
#include "id_api.h"
#include "id_cfg.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static ID_Sumo_t IdentifySumo(void);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static ID_Sumo_t currID = ID_SUMO_NONE;
static const ID_Cfg_t *idCfg = NULL;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static ID_Sumo_t IdentifySumo(void)
{
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



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
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
#endif /* !MASTER_ID_C_ */
