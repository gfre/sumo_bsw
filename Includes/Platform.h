/***********************************************************************************************//**
 * @file		Platform.h
 * @ingroup		incl
 * @brief 	 	Basic configuration macros of the Sumo BSW software
 *
 * This header file provides basic configuration macros for the preprocessor within the Sumo BSW
 * software. They define the features and supported hardware components of the Sumo robot.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	03.01.2017
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef PLATFORM_H_
#define PLATFORM_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Cpu.h" 		/* for PEcfg_FreeMASTER */
#include <stddef.h> 	/* for size_t */


/*======================================= >> #DEFINES << =========================================*/
#define SW_API_VERSION      '1'
#define SW_MAJOR_VERSION    '5'
#define SW_MINOR_VERSION    '2'
#define SW_YEAR             "2018"

#define SW_VERSION_CHAR_ARRAY   SW_API_VERSION, '.', SW_MAJOR_VERSION, '.', SW_MINOR_VERSION,'-'



#define KEY_PRESSED_NOTIFICATION_VALUE        	(0x01u)
#define KEY_RELEASED_NOTIFICATION_VALUE       	(0x02u)
#define KEY_PRESSED_LONG_NOTIFICATION_VALUE   	(0x04u)
#define KEY_RELEASED_LONG_NOTIFICATION_VALUE  	(0x08u)

#define CAU_SUMO_PLT_MOTOR_LEFT_INVERTED 		(TRUE)
#define CAU_SUMO_PLT_MOTOR_RIGHT_INVERTED 		(TRUE)

#define CAU_SUMO_PLT_NUM_OF_REFL_SENSORS		(0x06u)

#endif /* !PLATFORM_H_ */
