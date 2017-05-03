/*******************************************************************************
 * @brief 	Simple framework for custom demo application software.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date 	14.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements a simple framework for custom applications software
 * for education at Univeristy Kiel.
 *
 * ==============================================================================
 */

#ifndef DAPP_H_
#define DAPP_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "rte.h"

#ifdef MASTER_DAPP_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/* #define STUD_MACRO (0x01u) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/* typedef unit8 StudType_t; */



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


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !DAPP_H_ */
