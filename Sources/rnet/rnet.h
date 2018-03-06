/***********************************************************************************************//**
 * @file		rnet.h
 * @ingroup		rnet
 * @brief 		Interface of the SWC @a RNet for initialisation- and runtime-calls.
 *
 * This header file provides the internal interface between the SWC @ref rnet and the
 * SWC @ref task which runs the initialisation and periodic main function within a FreeRTOS task.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.01.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef RNET_H_
#define RNET_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_RNET_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup rnet
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the SWC @ref rnet
 */
#define RNET_SWC_STRING ("RNet")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Driver de-initialization
 */
EXTERNAL_ void RNET_Deinit(void);

/**
 * @brief Driver initialization
 */
EXTERNAL_ void RNET_Init(const void *pvPar_);

/**
 * @brief Driver main function
 */
EXTERNAL_ void RNET_MainFct(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* RNET_APPL_H_ */
