/***********************************************************************************************//**
 * @file		refl_cfg.h
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

#ifndef REFL_CFG_H_
#define REFL_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "refl_api.h"



#ifdef MASTER_refl_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup refl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This function returns the reference to the configuration of the reflectance sensor array
 * @return reference to the configuration
 */
EXTERNAL_ const Refl_Cfg_t *Get_pReflCfg(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !REFL_CFG_H_ */
