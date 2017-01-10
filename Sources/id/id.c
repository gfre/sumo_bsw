/*
 id.c
 *
 *  Created on: 09.01.2017
 *      Author: gefr
 */


#define MASTER_ID_C_
#include "Platform.h"
#include "KIN1.h"
#include "id.h"
#include "id_cfg.h"

static ID_Sumos currSumo = ID_SUMO_NONE;


static ID_Sumos IdentifySumo(void) {
  uint8_t res;
  KIN1_UID id;
  ID_Sumos i, sumo;

  sumo = ID_SUMO_UNKNOWN;
  res = KIN1_UIDGet(&id);
  if (res==ERR_OK) {
    for(i=(ID_Sumos)0; i<Get_ID_Cfg()->IdNum && ID_SUMO_UNKNOWN==sumo; i++) {
      if (KIN1_UIDSame(&id, Get_ID_Cfg()->Ids)) {
        sumo = i; /* found it */
      }
    }
  }
  return sumo;
}

ID_Sumos ID_WhichSumo(void) {
  if (currSumo==ID_SUMO_NONE) {
    /* not checked ID, try to find matching sumo */
    currSumo = IdentifySumo();
  }
  return currSumo;
}

void ID_Deinit(void) {
  currSumo = ID_SUMO_NONE;
}

void ID_Init(void) {
  currSumo = ID_SUMO_NONE;
}

#ifdef MASTER_ID_C_
#undef MASTER_ID_C_
#endif
