/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file appl.h
 * 
 *==================================================================================================
 */


#ifndef APPL_H_
#define APPL_H_

/*======================================= >> #INCLUDES << ========================================*/


#ifdef MASTER_appl_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#define APPL_SWC_STRING ("appl")


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Init function of the software component 'application'
 * Initialises the state machine which handles the application software
 */
EXTERNAL_ void APPL_Init(void);


/*!
 * @brief Main function of the software component 'state'
 * Runs the state machine of the main state
 */
EXTERNAL_ void APPL_MainFct(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !APPL_H_ */
