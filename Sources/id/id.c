/*
 id.c
 *
 *  Created on: 09.01.2016
 *      Author: gefr
 */


#define MASTER_ID_C_
#include "Platform.h"
#include "KIN1.h"
#include "id.h"
#include "id_cfg.h"

static ID_Robots currRobot = ID_ROBO_NONE;


static ID_Robots IdentifyRobot(void) {
  uint8_t res;
  KIN1_UID id;
  ID_Robots i, robo;

  robo = ID_ROBO_UNKNOWN;
  res = KIN1_UIDGet(&id);
  if (res==ERR_OK) {
    for(i=(ID_Robots)0; i<Get_ID_Cfg()->IdNum; i++) {
      if (KIN1_UIDSame(&id, Get_ID_Cfg()->Ids)) {
        robo = i; /* found it */
        break; /* get out of for() loop */
      }
    }
  }
  return robo;
}

ID_Robots ID_WhichRobot(void) {
  if (currRobot==ID_ROBO_NONE) {
    /* not checked ID, try to find matching robot */
    currRobot = IdentifyRobot();
  }
  return currRobot;
}

void ID_Deinit(void) {
  currRobot = ID_ROBO_NONE;
}

void ID_Init(void) {
  currRobot = ID_ROBO_NONE;
}

#ifdef MASTER_ID_C_
#undef MASTER_ID_C_
#endif
