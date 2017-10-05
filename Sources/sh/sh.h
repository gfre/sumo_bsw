/***********************************************************************************************//**
 * @file		sh.h
 * @ingroup		sh
 * @brief 		Interface of the SWC @a Shell for initialisation- and runtime-calls.
 *
 * This header file provides the internal interface between the SWC @ref sh and the
 * SWC @ref task which runs the initialisation and periodic main function within a FreeRTOS task.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	02.01.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef SH_H_
#define SH_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_SH_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup sh
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/** String identification of the SWC @ref sh */
#define SH_SWC_STRING ("shell")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Shell initialization
 */
EXTERNAL_ void SH_Init(void);

/**
 * @brief Shell main function
 */
EXTERNAL_ void SH_MainFct(void);

/**
 * @brief Sehll de-initialization
 */
EXTERNAL_ void SH_Deinit(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !SH_H_ */
