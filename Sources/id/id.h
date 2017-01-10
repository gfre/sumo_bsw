/*
 * id.h
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

#define MAX_NUMBER_OF_SUMOS    (25)
#define MAX_ID_OF_SUMOS        (MAX_NUMBER_OF_SUMOS-(1))

typedef struct ID_Cfg_
{
	const KIN1_UID *Ids;
	uint8_t IdNum;
}ID_Cfg_t;

typedef enum { /* do *not* change order of enumeration, they are used internally for a table index */
  ID_SUMO_MAX_ID=MAX_ID_OF_SUMOS, 	        /* max custom ID of sumo */
  ID_SUMO_UNKNOWN,         					/* unknown robot, unknown custom ID */
  ID_SUMO_NONE             					/* initialization value, used internally */
} ID_Sumos;

ID_Sumos ID_WhichSumo(void);

void ID_Deinit(void);

void ID_Init(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !ID_H */

