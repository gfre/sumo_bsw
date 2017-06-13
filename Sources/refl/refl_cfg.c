/***********************************************************************************************//**
 * @file		refl_cfg.c
 * @ingroup		refl
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	13.06.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_refl_cfg_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "refl_cfg.h"



/*======================================= >> #DEFINES << =========================================*/
/**
 * Values below this are not added to the weighted sum
 */
#define REFL_MIN_NOISE_VAL      (0x80u)

/**
 * Minimum value indicating a line
 */
#define REFL_MIN_LINE_VAL       (0x120u)

/**
 * Timeout period when no reflection is considered.
 * @warning Has to be smaller than the timeout period of the corresponding LDD TimerUnit.
 */
#define REFL_SENSOR_TIMEOUT_US  (3500u)


/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static Refl_Cfg_t reflCfg = {
		REFL_MIN_NOISE_VAL, REFL_MIN_LINE_VAL, REFL_LINE_BLACK, REFL_SENSOR_TIMEOUT_US,
};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
const Refl_Cfg_t *Get_pReflCfg(void)	{return &reflCfg;}



#ifdef MASTER_refl_cfg_C_
#undef MASTER_refl_cfg_C_
#endif /* !MASTER_refl_cfg_C_ */
