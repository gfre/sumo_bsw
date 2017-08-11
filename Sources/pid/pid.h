/***********************************************************************************************//**
 * @file		pid.h
 * @ingroup		pid
 * @brief 		Interface of the SWC @a PID for initialisation call
 *
 * This header file provides the internal interface between the SWC @ref pid and the
 * SWC @ref appl which runs the initialisation within its STARTUP state.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.01.2017
 *
  * @note Interface for BSW-specific use only
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#ifndef PID_H_
#define PID_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_pid_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup pid
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Driver initialization
 */
void PID_Init(void);

void PID_Init2(PID_Plant_t* plant_);

void PID_Deinit2(PID_Plant_t* plant_);

/**
 * @brief Driver de-initialization
 */
void PID_Deinit(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* PID_H_ */
