/***********************************************************************************************//**
 * @file		dapp.h
 * @ingroup		dapp
 * @brief 		Interface of the SWC *Demo Application* for initialisation- and runtime-calls
 *
 * This header file provides the internal interface between the SWC @ref dapp and the SWC @ref appl
 * which runs the initialisation within its INIT state and the main routine within its NORMAL state.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	03.05.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef DAPP_H_
#define DAPP_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "rte.h"

#ifdef MASTER_DAPP_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup dapp
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Init function of the software component @ref dapp
 * Initialises demo application
 */
EXTERNAL_ void DAPP_Init(void);



/**
 * @brief Main function of the software component @ref dapp
 * Calls main routine of demo application
 */
EXTERNAL_ void DAPP_Main(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !DAPP_H_ */
