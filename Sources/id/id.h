/***********************************************************************************************//**
 * @file		id.h
 * @ingroup		id
 * @brief 		Interface of the SWC @a Identification for (de-)initialisation-call
 *
 * This header file provides the internal interface between the SWC @ref id and the
 * SWC @ref appl which runs the initialisation within its STARTUP state.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.01.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef ID_H
#define ID_H

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_ID_C_
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



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This function terminates the ID software component
 */
void ID_Deinit(void);

/**
 * @brief This function initialises the ID software component
 */
void ID_Init(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* ID_H */
