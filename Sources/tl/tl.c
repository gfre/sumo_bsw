/***********************************************************************************************//**
 * @file		tl.c
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	S. Helling stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.08.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_tl_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tl.h"
#include "tl_api.h"
#include "pid_api.h"
#include "tacho.h"



/*======================================= >> #DEFINES << =========================================*/
#define TL_CONDITIONAL_RETURN(condVar_, trueVal_, falseVal_) ( (TRUE == condVar_)?(trueVal_) : (falseVal_) )
#define TL_CalcFilteredVal( plant_,  result_ )    ( PID( plant_,  result_ ) )

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static PID_Itm_t* TL_LftPlnt  = NULL;
static PID_Itm_t* TL_RghtPlnt = NULL;
static int32_t TL_CurLftSpd = 0, TL_CurRghtSpd = 0;
static int32_t TL_CurLftPos = 0, TL_CurRghtPos = 0; //scaled to 1000



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
int32_t TL_Get_Speed(bool isLeft_)
{
	return TL_CONDITIONAL_RETURN(isLeft_, TL_CurLftSpd, TL_CurRghtSpd);
}

StdRtn_t TL_Read_CurLftPos(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != result_)
	{
		*result_ = TL_CurLftPos/1000;
		retVal 	= ERR_OK;
	}
	return retVal;
}

StdRtn_t TL_Read_CurRghtPos(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != result_)
	{
		*result_ = TL_CurRghtPos/1000;
		retVal 	= ERR_OK;
	}
	return retVal;
}

void TL_Init()
{
	TL_LftPlnt  = &(Get_pTLCfg()->pItmTbl[TL_LFT_SPD_EST]);
	TL_RghtPlnt = &(Get_pTLCfg()->pItmTbl[TL_RGHT_SPD_EST]);

	if(NULL != TL_LftPlnt && NULL != TL_RghtPlnt)
	{
		TL_LftPlnt->integralVal  = 0;
		TL_LftPlnt->lastError    = 0;
		TL_LftPlnt->Saturation   = PID_NO_SAT;

		TL_RghtPlnt->integralVal = 0;
		TL_RghtPlnt->lastError   = 0;
		TL_RghtPlnt->Saturation  = PID_NO_SAT;
	}
	else
	{
		/* error handling */
	}
}

void TL_Main()
{
	StdRtn_t retVal = ERR_OK;
	retVal |= TL_CalcFilteredVal(TL_LftPlnt,  &TL_CurLftSpd);
	retVal |= TL_CalcFilteredVal(TL_RghtPlnt, &TL_CurRghtSpd);
	TL_CurLftPos  += TL_CurLftSpd * TACHO_SAMPLE_PERIOD_MS;
	TL_CurRghtPos += TL_CurRghtSpd * TACHO_SAMPLE_PERIOD_MS;

	if (ERR_OK != retVal)
	{
		/* error handling */
	}
}

void TL_Deinit()
{
	TL_CurLftSpd  = 0;
	TL_CurRghtSpd = 0;
	TL_CurLftPos  = 0;
	TL_CurRghtPos = 0;
}

#ifdef MASTER_tl_C_
#undef MASTER_tl_C_
#endif /* !MASTER_tl_C_ */
