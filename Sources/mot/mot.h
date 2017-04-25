/***********************************************************************************************//**
 * @file		mot.h
 * @ingroup		mot
 * @brief 		Interface of the SWC @a Motor Driver for initialisation call
 *
 * This header file provides the internal interface between the SWC @ref motor and the
 * SWC @ref drv which runs the initialisation within its initialisation routine.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.01.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#ifndef MOT_H_
#define MOT_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"



#ifdef MASTER_mot_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup mot
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * This function de-initialises the motor driver software component
 */
void MOT_Deinit(void);

/**
 * This function initialises the motor driver software component
 */
void MOT_Init(void);


/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MOT_API_H_ */

