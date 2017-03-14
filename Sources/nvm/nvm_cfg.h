/***************************************************************************************************
 * @brief 	Configuration of the Non-Volatile-Memory (NVM) storage.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This provides the configuration to store and retrieve data from the on-chip memory.
 *
 * =================================================================================================
 */

#ifndef NVM_CFG_H_
#define NVM_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_NVM_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_CFG_H_ */
