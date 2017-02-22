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

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_ID_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This function de-initialises the ID software component
 */
void ID_Deinit(void);

/**
 * @brief This function initialises the ID software component
 */
void ID_Init(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* ID_H */

