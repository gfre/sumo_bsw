/***********************************************************************************************//**
 * @file		appl.h
 * @ingroup		appl
 * @brief 		Header of the SWC @a Application for internal use
 *
 * This header file provides the internal interface between the SWC @a APPL and the
 * SWC @a TASK which runs the initialisation and periodic main function within a FreeRTOS task.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @note Interface for specific BSW use only
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
