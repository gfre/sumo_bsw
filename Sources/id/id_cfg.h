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

#include "Platform.h"
#include "id.h"

#ifdef MASTER_ID_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif



EXTERNAL_ const ID_Cfg_t *Get_ID_Cfg(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* ID_CFG_H */
