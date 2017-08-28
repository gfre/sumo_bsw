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

/* Filter headers API + function calls */
#include "kf.h"
#include "kf_api.h"
#include "maf.h"
#include "maf_api.h"
#include "tl.h"
#include "tl_api.h"
#include "tl_cfg.h"


/*======================================= >> #DEFINES << =========================================*/
// TODO
// remove FILTER ID work with index  and names/ short names instead
// order of table shall be priority


/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static TACHO_FltrItm_t fltrItms[] =
{
		{KF_FILTER_STRING, TRUE,  KF_Init,  KF_Main,    KF_Deinit,  NULL, KF_Get_Speed},
		{TL_FILTER_STRING, FALSE, TL_RunStartup, TL_CalcSpd, TL_RunStartup, NULL,  TL_Get_Speed},
		{MAF_FILTER_STRING,FALSE, MAF_Init, MAF_Main,   MAF_Deinit, MAF_UpdateRingBuffer, MAF_Get_Speed},
};


static TACHO_FltrItmTbl_t ftlrTbl =
{
		fltrItms,
		sizeof(fltrItms)/sizeof(fltrItms[0]),
};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
TACHO_FltrItmTbl_t* Get_pFltrTbl(void) {return &ftlrTbl;}



#ifdef MASTER_tacho_cfg_C_
#undef MASTER_tacho_cfg_C_
#endif /* !MASTER_refl_cfg_C_ */
