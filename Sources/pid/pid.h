/***************************************************************************************************
 * @brief 	Implementation of PID controllers.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 		06.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements PID controllers for position and speed control of the sumo robots. It uses
 * the NVM software component for storing the controller parameters.
 *
 *==================================================================================================
 */


#ifndef PID_H_
#define PID_H_

/*======================================= >> #INCLUDES << ========================================*/


#ifdef MASTER_pid_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*! @brief Driver initialization */
void PID_Init(void);

/*! @brief Driver de-initialization */
void PID_Deinit(void);




#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* PID_H_ */
