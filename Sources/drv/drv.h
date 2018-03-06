/***********************************************************************************************//**
 * @file		drv.h
 * @ingroup		drv
 * @brief 		Interface of the SWC @a Drive for initialisation- and runtime-calls.
 *
 * This header file provides the internal interface between the SWC @ref drv and the
 * SWC @ref task which runs the initialisation and periodic main function within a FreeRTOS task.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.01.2017
 *
 * @note Interface for BSW-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef DRIVE_H_
#define DRIVE_H_

/*======================================= >> #INCLUDES << ========================================*/


#ifdef MASTER_drv_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup drv
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/** String identification of the SWC @ref drv */
#define DRV_SWC_STRING ("drive")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Driver initialization.
 */
void DRV_Init(const void *pvPar_);

/**
 * @brief Driver de-initialization.
 */
void DRV_DeInit(void);

/**
 * @brief Driver main function
 */
void DRV_MainFct(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* DRIVE_H_ */
