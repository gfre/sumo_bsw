/***********************************************************************************************//**
 * @file		tacho_cfg.h
 * @ingroup		tacho
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

#ifndef TACHO_CFG_H_
#define TACHO_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "tacho_api.h"



#ifdef MASTER_tacho_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup tacho
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ TACHO_Cfg_t* Get_pTachoCfg(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_CFG_H_ */
