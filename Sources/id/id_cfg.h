/*******************************************************************************
 * @brief		Config helper for the ID module.
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

#ifndef ID_CFG_H
#define ID_CFG_H

/*======================================= >> #INCLUDES << ========================================*/
#include "KIN1.h"


#ifdef MASTER_ID_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct ID_Cfg_s
{
	const KIN1_UID *uids;	/**< UID table of the MCUs */
	uint8_t uidNum;			/**< Number of UIDs in the table */
}ID_Cfg_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This function returns configuration table of the MCU UIDs
 * @return configuration table of the UIDs
 */
EXTERNAL_ const ID_Cfg_t *Get_IDCfg(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* ID_CFG_H */
