/***********************************************************************************************//**
 * @file		tacho_cfg.c
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

#define MASTER_tacho_cfg_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tacho_cfg.h"
#include "kf.h"
#include "kf_api.h"
#include "tacho.h"



/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static TACHO_Filter_t FilterTbl[] =
{
		{MAF_FILTER_STRING, MOVING_AVERAGE_FILTER, FALSE, FALSE, MAF_Init, MAF_Main, MAF_Deinit, MAF_Get_Speed},
		{KF_FILTER_STRING,  KALMAN_FILTER,  	   FALSE, TRUE,  KF_Init,  KF_Main,  KF_Deinit,  KF_GetSpeed},
};


static TACHO_Cfg_t tachoCfg =
{
		FilterTbl,
		sizeof(FilterTbl)/sizeof(FilterTbl[0]),
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/

TACHO_Cfg_t* Get_pTachoCfg(void) {return &tachoCfg;}


#ifdef MASTER_tacho_cfg_C_
#undef MASTER_tacho_cfg_C_
#endif /* !MASTER_refl_cfg_C_ */
