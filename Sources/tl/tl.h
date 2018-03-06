/***********************************************************************************************//**
 * @file		tl.h
 * @ingroup		tl
 * @brief 		Interface of the SWC @ref tl for initialisation- and runtime-calls.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.08.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 **************************************************************************************************/

#ifndef TL_H_
#define TL_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_tl_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup tl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the SWC @ref tl
 */
#define TL_SWC_STRING ("tracking loop")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Initialisation function of the software component @ref tl
 *
 * This function initialises the configured objects of the software component
 */
EXTERNAL_ void TL_Init(void);



/**
 * @brief Main function of the software component  @ref tl
 *
 *  This function runs the implementation of the tracking loop.
 */
EXTERNAL_ void TL_Main(void);



/**
 * @brief De-Initialisation function of the software component @ref tl
 *
 * This function de-initialises the configured objects of the software component
 */
EXTERNAL_ void TL_DeInit(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TL_H_ */
