/***********************************************************************************************//**
 * @file		pid_cfg.h
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	<I>. <Surname>, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.08.2017
 *
 * @copyright @<LGPLx_x>
 *
 ***************************************************************************************************/

#ifndef PID_CFG_H_
#define PID_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "pid_api.h"



#ifdef MASTER_pid_cfg_C_
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
EXTERNAL_ PID_PlantCfg_t* Get_pPidCfg(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_CFG_H_ */
