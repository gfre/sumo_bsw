/*
 * identify.h
 *
 *  Created on: 26.11.2015
 *      Author: tastyger
 */

#ifndef ID_H
#define ID_H

#include "Platform.h"
#include "KIN1.h"

#ifdef MASTER_ID_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


typedef struct ID_Cfg_
{
	const KIN1_UID *Ids;
	uint8_t IdNum;
}ID_Cfg_t;

typedef enum { /* do *not* change order of enumeration, they are used internally for a table index */
  ID_ROBO_UNKNOWN=26, /* unknown robot, unknown ID */
  ID_ROBO_NONE /* initialization value, used internally */
} ID_Robots;

ID_Robots ID_WhichRobot(void);

void ID_Deinit(void);

void ID_Init(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !ID_H */

