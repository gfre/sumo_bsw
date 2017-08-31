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

// TODO
// - TEST: Move TACHO to a separate task and run it faster than DRV

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
static StdRtn_t KF_Read_i16EstdVal(int16_t *pVal_, uint8_t idx_);
static StdRtn_t MAF_Read_i16FltrdVal(int16_t *pVal_, uint8_t idx_);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static TACHO_FltrItm_t fltrItms[] =
{
		{KF_SWC_STRING,  TRUE,  KF_Init,  KF_Main,  KF_Deinit,  NULL, KF_Read_i16EstdVal},
		{TL_SWC_STRING,  FALSE, TL_Init,  TL_Main,  TL_DeInit,  NULL, TL_Read_i16dFltrdValdt},
		{MAF_SWC_STRING, FALSE, MAF_Init, MAF_Main, MAF_Deinit, MAF_UpdateRingBuffer, MAF_Read_i16FltrdVal},
};


static TACHO_FltrItmTbl_t ftlrTbl =
{
		fltrItms,
		sizeof(fltrItms)/sizeof(fltrItms[0]),
};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
// TODO Workaround - Read API needs to be provided from KF and MAF
StdRtn_t KF_Read_i16EstdVal(int16_t *pVal_, uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != pVal_)
	{
		if(0 == idx_)
		{
			*pVal_ = KF_Get_Speed(TRUE);
		}
		else
		{
			*pVal_ = KF_Get_Speed(FALSE);
		}
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t MAF_Read_i16FltrdVal(int16_t *pVal_, uint8_t idx_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != pVal_)
	{
		if(0 == idx_)
		{
			*pVal_ = MAF_Get_Speed(TRUE);
		}
		else
		{
			*pVal_ = MAF_Get_Speed(FALSE);
		}
		retVal = ERR_OK;
	}
	return retVal;
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
TACHO_FltrItmTbl_t* Get_pFltrTbl(void) {return &ftlrTbl;}



#ifdef MASTER_tacho_cfg_C_
#undef MASTER_tacho_cfg_C_
#endif /* !MASTER_refl_cfg_C_ */
