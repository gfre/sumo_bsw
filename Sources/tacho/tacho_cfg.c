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



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static int32_t TL_Get_Speed(bool isLeft_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static TACHO_FltrItm_t fltrItms[] =
{
		{KF_SWC_STRING,  TRUE,  KF_Init,  KF_Main,  KF_Deinit,  NULL, KF_Get_Speed},
		{TL_SWC_STRING,  FALSE, TL_Init,  TL_Main,  TL_DeInit,  NULL, TL_Get_Speed},
		{MAF_SWC_STRING, FALSE, MAF_Init, MAF_Main, MAF_Deinit, MAF_UpdateRingBuffer, MAF_Get_Speed},
};


static TACHO_FltrItmTbl_t ftlrTbl =
{
		fltrItms,
		sizeof(fltrItms)/sizeof(fltrItms[0]),
};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static int32_t TL_Get_Speed(bool isLeft_)
{
	int32_t tmp = 0;
	if( TRUE == isLeft_)
	{
		(void)TL_Read_i32dFltrdValdt(&tmp, 0);
	}
	else
	{
		(void)TL_Read_i32dFltrdValdt(&tmp, 1);
	}

	return tmp;
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
TACHO_FltrItmTbl_t* Get_pFltrTbl(void) {return &ftlrTbl;}



#ifdef MASTER_tacho_cfg_C_
#undef MASTER_tacho_cfg_C_
#endif /* !MASTER_refl_cfg_C_ */
