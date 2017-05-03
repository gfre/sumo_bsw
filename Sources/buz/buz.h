/***********************************************************************************************//**
 * @file		buz.h
 * @ingroup		buz
 * @brief 		Interface of the SWC @a Buzzer for initialisation- and runtime-calls
 *
 * This header file provides the internal interface between the SWC @ref buz and the
 * SWC @ref appl which runs the initialisation within its INIT state.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.01.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef BUZ_H_
#define BUZ_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_buz_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup buz
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Initialization of the driver
 */
EXTERNAL_ void BUZ_Init(void);


/**
 * @brief De-initialization of the driver
 */
EXTERNAL_ void BUZ_Deinit(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* BUZ_H_ */
