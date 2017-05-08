/***********************************************************************************************//**
 * @file		appl.h
 * @ingroup		appl
 * @brief 		Interface of the SWC @a Application for initialisation- and runtime-calls
 *
 * This header file provides the internal interface between the SWC @ref appl and the
 * SWC @ref task which runs the initialisation and periodic main function within a FreeRTOS task.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#ifndef APPL_H_
#define APPL_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_appl_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup appl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/** String identification of the SWC @ref appl */
#define APPL_SWC_STRING ("appl")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Init function of the software component 'application'
 *
 * Initialises the state machine which handles the application software
 */
EXTERNAL_ void APPL_Init(void);


/**
 * @brief Main function of the software component 'state'
 *
 * Runs the state machine of the main state
 */
EXTERNAL_ void APPL_MainFct(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !APPL_H_ */
