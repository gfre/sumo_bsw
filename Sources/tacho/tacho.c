/***********************************************************************************************//**
 * @file		tacho.c
 * @ingroup		tacho
 * @brief 		Implementation of a tachometer for speed calculation of two instances
 *
 * This module implements a tachometer component which calculates the speed based on quadrature
 * counters for up to two speed sources. The sign of the calculated speed signal indicates the
 * direction of movement. Furthermore, it provides a moving average filter to smoothing the speed
 * signal using a ring buffer for data collection. The module uses the firmware components
 * @a Q4C for both, left- and right-hand side, speed signals.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	30.03.2017
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
static int32_t TACHO_unfltrdLeftSpd = 0, TACHO_unfltrdRightSpd = 0;
static int32_t TACHO_fltrdLeftSpd 	= 0, TACHO_fltrdRightSpd   = 0;
static int32_t TACHO_curLeftPos     = 0, TACHO_curRightPos     = 0;
static int32_t TACHO_oldLeftPos     = 0, TACHO_oldRightPos     = 0;

static TACHO_Filter_t *pActiveFilter = NULL;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t TACHO_Read_CurLeftPos(int32_t* pos_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != pos_)
	{
		*pos_  = TACHO_curLeftPos;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurRightPos(int32_t* pos_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != pos_)
	{
		*pos_  = TACHO_curRightPos;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurUnfltrdLeftSpd(int32_t* speed_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != speed_)
	{
		*speed_  = TACHO_unfltrdLeftSpd;
		retVal   = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurUnfltrdRightSpd(int32_t* speed_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != speed_)
	{
		*speed_  = TACHO_unfltrdRightSpd;
		retVal   = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurFltrdLeftSpd(int32_t* speed_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != speed_)
	{
		*speed_ = TACHO_fltrdLeftSpd;
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_CurFltrdRightSpd(int32_t* speed_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != speed_)
	{
		*speed_ = TACHO_fltrdRightSpd;
		retVal  = ERR_OK;
	}
	return retVal;
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
	TACHO_oldLeftPos   = TACHO_curLeftPos;
	TACHO_oldRightPos  = TACHO_curRightPos;
	tempLeft  = Q4CLeft_GetPos();
	tempRight = Q4CRight_GetPos();
	TACHO_curLeftPos  = (int32_t)tempLeft;
	TACHO_curRightPos = (int32_t)tempRight;
	if(MOVING_AVERAGE_FILTER == pActiveFilter->FilterType)
	{
		MAF_UpdateRingBuffer(tempLeft, tempRight);
	}
	if(TRUE == pActiveFilter->isUsingSampledSpeed)
	{
		int32_t deltaLeft = 0, speedLeft = 0, deltaRight = 0, speedRight = 0;
		bool negLeft, negRight;
		deltaLeft = TACHO_oldLeftPos - TACHO_curLeftPos;
		if(deltaLeft < 0)
		{
			deltaLeft = -deltaLeft;
			negLeft   = TRUE;
		}
		else negLeft = FALSE;
		deltaRight = TACHO_oldRightPos - TACHO_curRightPos;
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
		TACHO_unfltrdLeftSpd  = -speedLeft;
		TACHO_unfltrdRightSpd = -speedRight;
	}
}



void TACHO_Deinit(void)
{
	/* nothing to do */
}

void TACHO_Init(void)
{
	TACHO_Set_FilterType(MOVING_AVERAGE_FILTER); //attention when changing! also change in pid_cfg.c!
}

void TACHO_Main(void)
{
	if( ( NULL != pActiveFilter ) && ( NULL != pActiveFilter->pFilterMainFct ) )
	{
		pActiveFilter->pFilterMainFct();
		TACHO_fltrdLeftSpd  = pActiveFilter->pGetSpeedFct(TRUE);
		TACHO_fltrdRightSpd = pActiveFilter->pGetSpeedFct(FALSE);
	}
	else
	{
		/* TODO error handling */
	}
}

void TACHO_Set_FilterType(TACHO_FilterType_t type_)
{
	TACHO_Cfg_t *tbl = NULL;
	tbl = Get_pTachoCfg();
	if( (NULL != pActiveFilter) && (NULL != pActiveFilter->pFilterDeinitFct) )
	{
		if(TRUE == pActiveFilter->isInitialized)
		{
			pActiveFilter->pFilterDeinitFct();
			if(TRUE == pActiveFilter->isUsingSampledSpeed)
			{
				TACHO_unfltrdLeftSpd  = 0;
				TACHO_unfltrdRightSpd = 0;
			}
			pActiveFilter->isInitialized = FALSE;
		}
	}

	pActiveFilter = &(tbl->pFilterTable[type_]);
	if( (NULL != pActiveFilter) && (NULL != pActiveFilter->pFilterInitFct) )
	{
		if(FALSE == pActiveFilter->isInitialized)
		{
			pActiveFilter->pFilterInitFct();
			pActiveFilter->isInitialized = TRUE;
		}
	}
	else
	{
		/* TODO error handling */
	}
}

TACHO_FilterType_t TACHO_Get_FilterType(void)
{
	return pActiveFilter->FilterType;
}

#ifdef MASTER_tacho_C_
#undef MASTER_tacho_C_
#endif /* !MASTER_tacho_C_ */
