/***********************************************************************************************//**
 * @file		buz_cfg.h
 * @ingroup		buz
 * @brief 		SWC-internal configuration interface of the SWC @a Buzzer
 *
 * This header file provides an internal interface within the software component SWC @a Buzzer
 * for the configuration of pre-defined melodies and tunes.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.02.2017
 *  
 * @note Interface for SWC-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef BUZ_CFG_H_
#define BUZ_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"



#ifdef MASTER_buz_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup buz
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct {
  uint16_t buzPeriodTicks;		/**< number of trigger ticks for a PWM period */
  uint16_t buzIterationCntr;	/**< number of iterations */
} BUZ_TrgInfo;

typedef struct {
  uint16_t freq; 				/**< frequency */
  uint16_t ms; 					/**< milliseconds */
} BUZ_Tune;

typedef struct {
  uint8_t idx; 					/**< current index */
  const uint8_t maxIdx; 		/**< maximum index */
  BUZ_TrgInfo trgInfo;
  const BUZ_Tune *melody;
} MelodyDesc;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ MelodyDesc *Get_BUZMelodies(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !BUZ_CFG_H_ */
