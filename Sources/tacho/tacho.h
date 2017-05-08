/***********************************************************************************************//**
 * @file		tacho.h
 * @ingroup		tacho
 * @brief 		Interface of the SWC @a Tacho for initialisation- and runtime-calls.
 *
 * This header file provides the internal interface between the SWC @ref tacho and the
 * SWC @ref task which runs the initialisation and periodic main function within a FreeRTOS task.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	30.03.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef TACHO_H_
#define TACHO_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_tacho_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup tacho
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the SWC @ref tacho
 */
#define TACHO_SWC_STRING ("Tacho")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Calculates the speed based on the position information from the encoder.
 */
void TACHO_CalcSpeed(void);

/**
 * @brief Sampling routine to calculate speed, must be called periodically with a fixed frequency.
 */
void TACHO_Sample(void);

/**
 * @brief De-initialization of the module
 */
void TACHO_Deinit(void);

/**
 * @brief Initialization of the module
 */
void TACHO_Init(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_H_ */
