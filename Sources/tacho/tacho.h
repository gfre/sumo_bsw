/***************************************************************************************************
 * @brief 	Implementation of the tachometer software component
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	30.03.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module calculates the speed based on the quadrature counter. It implements an moving average
 * filter for the speed signal based on a ring buffer.
 *
 *==================================================================================================
 */

#ifndef TACHO_H_
#define TACHO_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"

#ifdef MASTER_tacho_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#define TACHO_SWC_STRING ("tacho")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Returns the previously calculated speed of the motor.
 * @param isLeft TRUE for left speed, FALSE for right speed.
 * @return Actual speed value
 */
int32_t TACHO_GetSpeed(bool isLeft);

/*!
 * @brief Calculates the speed based on the position information from the encoder.
 */
void TACHO_CalcSpeed(void);

/*!
 * @brief Sampling routine to calculate speed, must be called periodically with a fixed frequency.
 */
void TACHO_Sample(void);

/*! @brief De-initialization of the module */
void TACHO_Deinit(void);

/*! @brief Initialization of the module */
void TACHO_Init(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* __TACHO_H_ */
