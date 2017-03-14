/***************************************************************************************************
 * @brief 	Implementation of the Non-Volatile-Memory (NVM) storage.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This provides an implementation to store and retrieve data from the on-chip memory.
 *
 * =================================================================================================
 */

#ifndef NVM_H_
#define NVM_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_NVM_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Init function of the software component 'non-volatile memory'
 * Initialises the NVM component - NVM is restored from ROM if it is not up2date
 */
EXTERNAL_ void NVM_Init(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_H_ */
