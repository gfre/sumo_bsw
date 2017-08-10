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
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "FRTOS1.h"
#include "CS1.h"



/*======================================= >> #DEFINES << =========================================*/
#define TACHO_CONDITIONAL_RETURN(condVar_, trueVal_, falseVal_) ( (TRUE == condVar_) ? (trueVal_) : (falseVal_) )


#define NOF_HISTORY (16U+1U)

/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static int32_t TACHO_unfltrdLeftSpeed = 0, TACHO_unfltrdRightSpeed = 0;
static int32_t TACHO_currLeftPos = 0, TACHO_currRightPos = 0;
static int32_t TACHO_oldLeftPos = 0, TACHO_oldRightPos = 0;

static TACHO_Filter_t *pActiveFilterCfg = NULL;


/*!< for better accuracy, we calculate the speed over some samples */
static volatile Q4CLeft_QuadCntrType MAF_LeftPosHistory[NOF_HISTORY], MAF_RightPosHistory[NOF_HISTORY];
/*!< position index in history */
static volatile uint8_t MAF_PosHistory_Index = 0;

static int32_t MAF_currLeftSpeed = 0, MAF_currRightSpeed = 0;
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


void MAF_Init(void)
{
	MAF_currLeftSpeed = 0;
	MAF_currRightSpeed = 0;
	MAF_PosHistory_Index = 0;
}
int32_t MAF_Get_Speed(bool isLeft_)
{
	return (int32_t)TACHO_CONDITIONAL_RETURN(isLeft_, MAF_currLeftSpeed, MAF_currRightSpeed);
}
void MAF_Main(void) {
	/* we calculate the speed as follow:
                              1000
  steps/sec =  delta * -----------------
                       samplePeriod (ms)
  As this function may be called very frequently, it is important to make it as efficient as possible!
	 */
	int32_t deltaLeft, deltaRight, newLeft, newRight, oldLeft, oldRight;
	int32_t speedLeft, speedRight;
	bool negLeft, negRight;
	CS1_CriticalVariable()

	CS1_EnterCritical();
	oldLeft = (int32_t)MAF_LeftPosHistory[MAF_PosHistory_Index]; /* oldest left entry */
	oldRight = (int32_t)MAF_RightPosHistory[MAF_PosHistory_Index]; /* oldest right entry */
	if (MAF_PosHistory_Index==0) { /* get newest entry */
		newLeft = (int32_t)MAF_LeftPosHistory[NOF_HISTORY-1];
		newRight = (int32_t)MAF_RightPosHistory[NOF_HISTORY-1];
	} else{
		newLeft = (int32_t)MAF_LeftPosHistory[MAF_PosHistory_Index-1];
		newRight = (int32_t)MAF_RightPosHistory[MAF_PosHistory_Index-1];
	}
	CS1_ExitCritical();
	deltaLeft = oldLeft-newLeft; /* delta of oldest position and most recent one */
	/* use unsigned arithmetic */
	if (deltaLeft < 0) {
		deltaLeft = -deltaLeft;
		negLeft = TRUE;
	} else {
		negLeft = FALSE;
	}
	deltaRight = oldRight-newRight; /* delta of oldest position and most recent one */
	/* use unsigned arithmetic */
	if (deltaRight < 0) {
		deltaRight = -deltaRight;
		negRight = TRUE;
	} else {
		negRight = FALSE;
	}
	/* calculate speed. this is based on the delta and the time (number of samples or entries in the history table) */
	speedLeft  = (int32_t)(deltaLeft*1000U/(TACHO_SAMPLE_PERIOD_MS*(NOF_HISTORY-1)));
	if (negLeft) {
		speedLeft  = -speedLeft;
	}
	speedRight = (int32_t)(deltaRight*1000U/(TACHO_SAMPLE_PERIOD_MS*(NOF_HISTORY-1)));
	if (negRight) {
		speedRight = -speedRight;
	}
	MAF_currLeftSpeed     = -speedLeft;  /* store current speed in global variable */
	MAF_currRightSpeed    = -speedRight; /* store current speed in global variable */
}


void TACHO_Sample(void) {
	static int cnt = 0;

	/* get called from the RTOS tick counter. Divide the frequency. */
	cnt += portTICK_PERIOD_MS;
	if (cnt < TACHO_SAMPLE_PERIOD_MS) { /* sample only every TACHO_SAMPLE_PERIOD_MS */
		return;
	}
	cnt = 0; /* reset counter */

	/* left */
	TACHO_oldLeftPos  = TACHO_currLeftPos;
	TACHO_oldRightPos = TACHO_currRightPos;
	MAF_LeftPosHistory[MAF_PosHistory_Index]  = Q4CLeft_GetPos();
	MAF_RightPosHistory[MAF_PosHistory_Index] = Q4CRight_GetPos();
	TACHO_currLeftPos  = (int32_t)MAF_LeftPosHistory[MAF_PosHistory_Index];
	TACHO_currRightPos = (int32_t)MAF_RightPosHistory[MAF_PosHistory_Index];
	MAF_PosHistory_Index++;
	if (MAF_PosHistory_Index >= NOF_HISTORY) {
		MAF_PosHistory_Index = 0;
	}

	if(TRUE == pActiveFilterCfg->isUsingSampledSpeed)
	{
		int32_t deltaLeft = 0, speedLeft = 0, deltaRight = 0, speedRight = 0;
		bool negLeft, negRight;
		deltaLeft = TACHO_oldLeftPos - TACHO_currLeftPos;
		if(deltaLeft < 0)
		{
			deltaLeft = -deltaLeft;
			negLeft = TRUE;
		}
		else negLeft = FALSE;

		deltaRight = TACHO_oldRightPos - TACHO_currRightPos;
		if(deltaRight < 0)
		{
			deltaRight = -deltaRight;
			negRight = TRUE;
		}
		else negRight = FALSE;
		speedLeft =(int32_t)(deltaLeft*1000U/(TACHO_SAMPLE_PERIOD_MS));
		if(TRUE == negLeft) speedLeft = -speedLeft;
		speedRight =(int32_t)(deltaRight*1000U/(TACHO_SAMPLE_PERIOD_MS));
		if(TRUE == negRight) speedRight = -speedRight;
		TACHO_unfltrdLeftSpeed = -speedLeft;
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
	if( ( NULL != pActiveFilterCfg ) && ( NULL != pActiveFilterCfg->pFilterMainFct ) )
	{
		pActiveFilterCfg->pFilterMainFct();
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
	pActiveFilterCfg = &(tbl->pFilterTable[type_]);
	if( (NULL != pActiveFilterCfg) && ( NULL != pActiveFilterCfg->pFilterInitFct ) )
	{
		if(FALSE == pActiveFilterCfg->isInitialized)
		{
			pActiveFilterCfg->pFilterInitFct();
			pActiveFilterCfg->isInitialized = TRUE;
		}
	}
	else
	{
		/* TODO error handling */
	}
}

TACHO_FilterType_t TACHO_Get_FilterType(void)
{
	return pActiveFilterCfg->FilterType;
}

#ifdef MASTER_tacho_C_
#undef MASTER_tacho_C_
#endif /* !MASTER_tacho_C_ */
