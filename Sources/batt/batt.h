/***********************************************************************************************//**
 * @file		batt.h
 * @ingroup		batt
 * @brief 		Header of the SWC @a Battery for internal use
 *
 * This header file provides the internal interface between the SWC @a BATT and the
 * SWC @a APPL which runs the initialisation within its INIT state.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date	09.01.2017
 *
 * @note Interface for specific BSW use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef BATT__H_
#define BATT__H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_batt_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup batt
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Module Initialization.
 */
void BATT_Init(void);

/**
 * @brief Module De-initialization.
 */
void BATT_Deinit(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* BATT_H_ */
