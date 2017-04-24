/***********************************************************************************************//**
 * @file		id_cfg.h
 * @ingroup		id
 * @brief 		SWC-internal configuration interface of the SWC @a Identification
 *
 * This header file provides an internal interface within the software component SWC @ref id
 * for the configuration of the unique identifiers (UIDs) of the entire fleet of robots.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 		11.01.2017
 *
 * @note Interface for SWC-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef ID_CFG_H
#define ID_CFG_H

/*======================================= >> #INCLUDES << ========================================*/
#include "KIN1.h"


#ifdef MASTER_ID_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup id
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @typedef ID_Cfg_t
 * @brief ID_Cfg_t is defined as the struct ID_Cfg_s
 *
 * @struct ID_Cfg_s
 * @brief Struct which holds the UID information.
 */
typedef struct ID_Cfg_s
{
	const KIN1_UID *uids;	/**< UID table of the MCUs */
	uint8_t uidNum;			/**< Number of UIDs in the table */
}ID_Cfg_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This function returns the reference to the configuration table of the unique identifiers
 * (UIDs).
 * @return pointer to the configuration table of the unique identifiers
 */
EXTERNAL_ const ID_Cfg_t *Get_IDCfg(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* ID_CFG_H */
