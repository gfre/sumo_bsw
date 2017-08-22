/***********************************************************************************************//**
 * @file		tacho.c
 * @ingroup		tacho
 * @brief 		Implementation of a tachometer for speed calculation of two instances
 *
 * This module implements a tachometer component which calculates the speed based on quadrature
 * counters for up to two speed sources. The sign of the calculated speed signal indicates the
 * direction of movement. Furthermore, it provides access to filter components defined in the
 * tacho_cfg.c file to smooth the velocity signal. By default, it uses a moving average filter
 * to calulate the velocity. It samples the current position using component @a Q4C for
 * both, left- and right-hand side. Sampling rate is defined by TACHO_SAMPLING_PERIOD_MS.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author  S. Helling,  stu112498@tf-uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	17.08.2017
 *
* @copyright @LGPL2_1
 *
 **************************************************************************************************/


#define MASTER_tacho_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tacho.h"
#include "tacho_api.h"
#include "maf.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "FRTOS1.h"
#include "ACon_Types.h"



/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static int32_t TACHO_UnfltrdLftSpd = 0, TACHO_UnfltrdRghtSpd = 0;
static int32_t TACHO_FltrdLftSpd   = 0, TACHO_FltrdRghtSpd   = 0;
static int32_t TACHO_CurLftPos     = 0, TACHO_CurRghtPos     = 0;
static int32_t TACHO_OldLftPos     = 0, TACHO_OldRghtPos     = 0;

static TACHO_Fltr_t *pActiveFltr = NULL;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t TACHO_Read_CurLftPos(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != result_)
	{
		*result_  = TACHO_CurLftPos;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurRghtPos(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != result_)
	{
		*result_  = TACHO_CurRghtPos;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurUnfltrdLftSpd(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != result_)
	{
		*result_  = TACHO_UnfltrdLftSpd;
		retVal   = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurUnfltrdRghtSpd(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != result_)
	{
		*result_  = TACHO_UnfltrdRghtSpd;
		retVal   = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurFltrdLftSpd(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != result_)
	{
		*result_ = TACHO_FltrdLftSpd;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurFltrdRghtSpd(int32_t* result_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != result_)
	{
		*result_ = TACHO_FltrdRghtSpd;
		retVal  = ERR_OK;
	}
	return retVal;
}

void TACHO_Set_FltrType(TACHO_FltrType_t type_)
{
	TACHO_Cfg_t *tbl = NULL;
	tbl = Get_pTachoCfg();
	if( (NULL != pActiveFltr) && (NULL != pActiveFltr->pFilterDeinitFct) )
	{
		if(TRUE == pActiveFltr->isInitialized)
		{
			pActiveFltr->pFilterDeinitFct();
			if(TRUE == pActiveFltr->isUsingSampledSpeed)
			{
				TACHO_UnfltrdLftSpd  = 0;
				TACHO_UnfltrdRghtSpd = 0;
			}
			pActiveFltr->isInitialized = FALSE;
		}
	}

	pActiveFltr = &(tbl->pFilterTable[type_]);
	if( (NULL != pActiveFltr) && (NULL != pActiveFltr->pFilterInitFct) )
	{
		if(FALSE == pActiveFltr->isInitialized)
		{
			pActiveFltr->pFilterInitFct();
			pActiveFltr->isInitialized = TRUE;
		}
	}
	else
	{
		/* TODO error handling */
	}
}

TACHO_FltrType_t TACHO_Get_FltrType(void)
{
	return pActiveFltr->FilterType;
}

void TACHO_Sample(void) {
	static int cnt = 0;
	Q4CLeft_QuadCntrType tempLeft  = 0;
	Q4CLeft_QuadCntrType tempRight = 0;
	/* get called from the RTOS tick counter. Divide the frequency. */
	cnt += portTICK_PERIOD_MS;
	if (cnt < TACHO_SAMPLE_PERIOD_MS) { /* sample only every TACHO_SAMPLE_PERIOD_MS */
		return;
	}
	cnt = 0; /* reset counter */

	/* left */
	TACHO_OldLftPos   = TACHO_CurLftPos;
	TACHO_OldRghtPos  = TACHO_CurRghtPos;
	tempLeft  = Q4CLeft_GetPos();
	tempRight = Q4CRight_GetPos();
	TACHO_CurLftPos  = (int32_t)tempLeft;
	TACHO_CurRghtPos = (int32_t)tempRight;
	if(MOVING_AVERAGE_FILTER == pActiveFltr->FilterType)
	{
		MAF_UpdateRingBuffer(tempLeft, tempRight);
	}
	if(TRUE == pActiveFltr->isUsingSampledSpeed)
	{
		int32_t deltaLeft = 0, speedLeft = 0, deltaRight = 0, speedRight = 0;
		bool negLeft, negRight;
		deltaLeft = TACHO_OldLftPos - TACHO_CurLftPos;
		if(deltaLeft < 0)
		{
			deltaLeft = -deltaLeft;
			negLeft   = TRUE;
		}
		else negLeft = FALSE;
		deltaRight = TACHO_OldRghtPos - TACHO_CurRghtPos;
		if(deltaRight < 0)
		{
			deltaRight = -deltaRight;
			negRight   = TRUE;
		}
		else negRight = FALSE;
		speedLeft =(int32_t)(deltaLeft*1000U/(TACHO_SAMPLE_PERIOD_MS));
		if(TRUE == negLeft) speedLeft = -speedLeft;
		speedRight =(int32_t)(deltaRight*1000U/(TACHO_SAMPLE_PERIOD_MS));
		if(TRUE == negRight) speedRight = -speedRight;
		TACHO_UnfltrdLftSpd  = -speedLeft;
		TACHO_UnfltrdRghtSpd = -speedRight;
	}
}



void TACHO_Deinit(void)
{
	/* nothing to do */
}

void TACHO_Init(void)
{
	TACHO_Set_FltrType(MOVING_AVERAGE_FILTER); //attention when changing! also change in pid_cfg.c!
}

void TACHO_Main(void)
{
	if( ( NULL != pActiveFltr ) && ( NULL != pActiveFltr->pFilterMainFct ) )
	{
		pActiveFltr->pFilterMainFct();
		TACHO_FltrdLftSpd  = pActiveFltr->pGetSpeedFct(TRUE);
		TACHO_FltrdRghtSpd = pActiveFltr->pGetSpeedFct(FALSE);
	}
	else
	{
		/* TODO error handling */
	}
}


#ifdef MASTER_tacho_C_
#undef MASTER_tacho_C_
#endif /* !MASTER_tacho_C_ */
