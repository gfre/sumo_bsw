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

typedef struct ID_Cfg_s
{
	const KIN1_UID *ids;
	uint8_t idNum;
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

#endif /* ID_H */

