/***********************************************************************************************//**
 * @file		tacho_cfg.c
 * @ingroup		tacho
 * @brief 		Component to sample current position of motors. Also provides interface to used Filter
 *
 * This component provides an interface between a filter component to estimate the current velocity
 * and the tacho.c file. New filter components can be added to the fltrTbl and must match the provided
 * form of a 'TACHO_Fltr_t' variable and can then be activated using the command line shell.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author  S. Helling,		  stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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
#include "maf.h"
#include "maf_api.h"
#include "tacho.h"
#include "tl.h"
#include "tl_api.h"



/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static TACHO_Fltr_t fltrTbl[] =
{
		{MAF_FILTER_STRING, MOVING_AVERAGE_FILTER, FALSE, FALSE, MAF_Init, MAF_Main, MAF_Deinit, MAF_Get_Speed},
		{KF_FILTER_STRING,  KALMAN_FILTER,  	   FALSE, TRUE,  KF_Init,  KF_Main,  KF_Deinit,  KF_Get_Speed},
		{TL_FILTER_STRING,	TRACKING_LOOP_FILTER,  FALSE, FALSE, TL_Init,  TL_Main,  TL_Deinit,  TL_Get_Speed},
};


static TACHO_Cfg_t tachoCfg =
{
		fltrTbl,
		sizeof(fltrTbl)/sizeof(fltrTbl[0]),
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/

TACHO_Cfg_t* Get_pTachoCfg(void) {return &tachoCfg;}


#ifdef MASTER_tacho_cfg_C_
#undef MASTER_tacho_cfg_C_
#endif /* !MASTER_refl_cfg_C_ */
