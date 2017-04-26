/***********************************************************************************************//**
 * @file		nvm.h
 * @ingroup		nvm
 * @brief 		Interface of the SWC @a NVM for initialisation call
 *
 * This header file provides the internal interface between the SWC @ref nvm and the
 * SWC @ref appl which runs the initialisation within its STARTUP state.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#ifndef NVM_H_
#define NVM_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_NVM_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup nvm
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Init function of the software component 'non-volatile memory'
 * Initialises the NVM component - NVM is restored from ROM if it is not up2date
 */
EXTERNAL_ void NVM_Init(void);


/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_H_ */
