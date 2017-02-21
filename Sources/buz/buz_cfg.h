/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	(c) 2017 Gerhard Freudenthaler, gefr@tf.uni-kiel.de, University Kiel
 * @date 	06.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file buz_cfg.h
 * 
 *==================================================================================================
 */


#ifndef BUZ_CFG_H_
#define BUZ_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"

#ifdef MASTER_buz_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct {
  uint16 buzPeriodTicks; /*!< number of trigger ticks for a PWM period */
  uint16 buzIterationCntr; /*!< number of iterations */
} BUZ_TrgInfo;

typedef struct {
  uint16 freq; /* frequency */
  uint16 ms; /* milliseconds */
} BUZ_Tune;

typedef struct {
  uint8 idx; /* current index */
  const uint8 maxIdx; /* maximum index */
  BUZ_TrgInfo trgInfo;
  const BUZ_Tune *melody;
} MelodyDesc;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ MelodyDesc *Get_BUZMelodies(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !BUZ_CFG_H_ */
