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



/*======================================= >> #DEFINES << =========================================*/
#define TACHO_CONDITIONAL_RETURN(condVar_, trueVal_, falseVal_) ( (TRUE == condVar_) ? (trueVal_) : (falseVal_) )




/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static int32_t TACHO_unfltrdLeftSpeed = 0, TACHO_unfltrdRightSpeed = 0;
static int32_t TACHO_currLeftPos      = 0, TACHO_currRightPos      = 0;
static int32_t TACHO_oldLeftPos       = 0, TACHO_oldRightPos       = 0;

static TACHO_Filter_t *pActiveFilter = NULL;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
int32_t TACHO_Get_CurrentPosition(bool isLeft_)
{
	return (int32_t)TACHO_CONDITIONAL_RETURN(isLeft_, TACHO_currLeftPos, TACHO_currRightPos);
}

int32_t TACHO_Get_UnfilteredSpeed(bool isLeft_)
{
	return (int32_t)TACHO_CONDITIONAL_RETURN(isLeft_, TACHO_unfltrdLeftSpeed, TACHO_unfltrdRightSpeed);
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
	TACHO_oldLeftPos   = TACHO_currLeftPos;
	TACHO_oldRightPos  = TACHO_currRightPos;
	tempLeft  = Q4CLeft_GetPos();
	tempRight = Q4CRight_GetPos();
	TACHO_currLeftPos  = (int32_t)tempLeft;
	TACHO_currRightPos = (int32_t)tempRight;
	if(MOVING_AVERAGE_FILTER == pActiveFilter->FilterType)
	{
		MAF_UpdateRingBuffer(tempLeft, tempRight);
	}
	if(TRUE == pActiveFilter->isUsingSampledSpeed)
	{
		int32_t deltaLeft = 0, speedLeft = 0, deltaRight = 0, speedRight = 0;
		bool negLeft, negRight;
		deltaLeft = TACHO_oldLeftPos - TACHO_currLeftPos;
		if(deltaLeft < 0)
		{
			deltaLeft = -deltaLeft;
			negLeft   = TRUE;
		}
		else negLeft = FALSE;
		deltaRight = TACHO_oldRightPos - TACHO_currRightPos;
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
		TACHO_unfltrdLeftSpeed  = -speedLeft;
		TACHO_unfltrdRightSpeed = -speedRight;
	}
}



void TACHO_Deinit(void)
{
	/* nothing to do */
}

void TACHO_Init(void)
{
	TACHO_Set_FilterType(MOVING_AVERAGE_FILTER);
}

void TACHO_Main(void)
{
	if( ( NULL != pActiveFilter ) && ( NULL != pActiveFilter->pFilterMainFct ) )
	{
		pActiveFilter->pFilterMainFct();
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
				TACHO_unfltrdLeftSpeed  = 0;
				TACHO_unfltrdRightSpeed = 0;
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
