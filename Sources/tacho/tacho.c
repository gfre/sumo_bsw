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


/**
 * Number of samples for speed calculation (>0):the more, the better, but the slower.
 */
#define NOF_HISTORY (16U+1U) 



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
/*!< for better accuracy, we calculate the speed over some samples */
static volatile Q4CLeft_QuadCntrType TACHO_LeftPosHistory[NOF_HISTORY], TACHO_RightPosHistory[NOF_HISTORY];
/*!< position index in history */
static volatile uint8_t TACHO_PosHistory_Index = 0;

static int32_t TACHO_currLeftSpeed = 0, TACHO_currRightSpeed = 0, TACHO_actual5MsSpeed = 0;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
int32_t TACHO_GetSpeed(bool isLeft) {
	if (isLeft) {
		return TACHO_currLeftSpeed;
	} else {
		return TACHO_currRightSpeed;
	}
}

int32_t TACHO_GetUnfilteredSpeed(bool isLeft_)
{
	int32_t actualSpeed = 0;
	if(TRUE == isLeft_)
	{
		actualSpeed = TACHO_actual5MsSpeed;
	}
	else
	{
		actualSpeed = TACHO_actual5MsSpeed;
	}
	return actualSpeed;
}

void TACHO_CalcSpeed(void) {
	/* we calculate the speed as follow:
                              1000         
  steps/sec =  delta * ----------------- 
                       samplePeriod (ms) 
  As this function may be called very frequently, it is important to make it as efficient as possible!
	 */
	int32_t deltaLeft, deltaRight, delta5Ms, secondNewestLeft, newLeft, newRight, oldLeft, oldRight;
	int32_t speedLeft, speedRight, actual5MsSpeed;
	bool negLeft, negRight, negActual;
	CS1_CriticalVariable()

	CS1_EnterCritical();
	oldLeft = (int32_t)TACHO_LeftPosHistory[TACHO_PosHistory_Index]; /* oldest left entry */
	oldRight = (int32_t)TACHO_RightPosHistory[TACHO_PosHistory_Index]; /* oldest right entry */
	if (TACHO_PosHistory_Index==0) { /* get newest entry */
		newLeft = (int32_t)TACHO_LeftPosHistory[NOF_HISTORY-1];
		newRight = (int32_t)TACHO_RightPosHistory[NOF_HISTORY-1];
		secondNewestLeft = (int32_t)TACHO_LeftPosHistory[NOF_HISTORY-2];
	} else{
		newLeft = (int32_t)TACHO_LeftPosHistory[TACHO_PosHistory_Index-1];
		newRight = (int32_t)TACHO_RightPosHistory[TACHO_PosHistory_Index-1];
		if(TACHO_PosHistory_Index == 1)
		{
			secondNewestLeft = (int32_t)TACHO_LeftPosHistory[NOF_HISTORY-1];
		}else
		{
			secondNewestLeft = (int32_t)TACHO_LeftPosHistory[TACHO_PosHistory_Index-2];
		}
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
	delta5Ms = secondNewestLeft-newLeft;
	if (delta5Ms < 0) {
		delta5Ms = -delta5Ms;
		negActual = TRUE;
	} else {
		negActual = FALSE;
	}
	/* calculate speed. this is based on the delta and the time (number of samples or entries in the history table) */
	speedLeft = (int32_t)(deltaLeft*1000U/(TACHO_SAMPLE_PERIOD_MS*(NOF_HISTORY-1)));
	if (negLeft) {
		speedLeft = -speedLeft;
	}
	speedRight = (int32_t)(deltaRight*1000U/(TACHO_SAMPLE_PERIOD_MS*(NOF_HISTORY-1)));
	if (negRight) {
		speedRight = -speedRight;
	}
	actual5MsSpeed = (int32_t)(delta5Ms*1000U/(TACHO_SAMPLE_PERIOD_MS));
	if (negActual) {
		actual5MsSpeed = -actual5MsSpeed;
	}
	TACHO_currLeftSpeed = -speedLeft; /* store current speed in global variable */
	TACHO_currRightSpeed = -speedRight; /* store current speed in global variable */
	TACHO_actual5MsSpeed = -actual5MsSpeed;
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
	TACHO_LeftPosHistory[TACHO_PosHistory_Index] = Q4CLeft_GetPos();
	TACHO_RightPosHistory[TACHO_PosHistory_Index] = Q4CRight_GetPos();
	TACHO_PosHistory_Index++;
	if (TACHO_PosHistory_Index >= NOF_HISTORY) {
		TACHO_PosHistory_Index = 0;
	}
}



void TACHO_Deinit(void) {
}

void TACHO_Init(void) {
	TACHO_currLeftSpeed = 0;
	TACHO_currRightSpeed = 0;
	TACHO_PosHistory_Index = 0;
}


#ifdef MASTER_tacho_C_
#undef MASTER_tacho_C_
#endif /* !MASTER_tacho_C_ */
