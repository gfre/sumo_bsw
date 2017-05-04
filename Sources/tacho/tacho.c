/***************************************************************************************************
 * @brief 	Implementation of the tachometer software component
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	30.03.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module calculates the speed based on the quadrature counter. It implements an moving average
 * filter for the speed signal based on a ring buffer.
 *
 *==================================================================================================
 */


#define MASTER_tacho_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tacho.h"
#include "tacho_api.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "CLS1.h"
#include "UTIL1.h"
#include "FRTOS1.h"
#include "CS1.h"



/*======================================= >> #DEFINES << =========================================*/
/*!< speed sample period in ms. Make sure that speed is sampled at the given rate. */
#define TACHO_SAMPLE_PERIOD_MS (5)     
/*!< number of samples for speed calculation (>0):the more, the better, but the slower. */
#define NOF_HISTORY (16U+1U) 



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
/*!< for better accuracy, we calculate the speed over some samples */
static volatile Q4CLeft_QuadCntrType TACHO_LeftPosHistory[NOF_HISTORY], TACHO_RightPosHistory[NOF_HISTORY];
/*!< position index in history */
static volatile uint8_t TACHO_PosHistory_Index = 0;

static int32_t TACHO_currLeftSpeed = 0, TACHO_currRightSpeed = 0;




/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
int32_t TACHO_GetSpeed(bool isLeft) {
	if (isLeft) {
		return TACHO_currLeftSpeed;
	} else {
		return TACHO_currRightSpeed;
	}
}

void TACHO_CalcSpeed(void) {
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
	oldLeft = (int32_t)TACHO_LeftPosHistory[TACHO_PosHistory_Index]; /* oldest left entry */
	oldRight = (int32_t)TACHO_RightPosHistory[TACHO_PosHistory_Index]; /* oldest right entry */
	if (TACHO_PosHistory_Index==0) { /* get newest entry */
		newLeft = (int32_t)TACHO_LeftPosHistory[NOF_HISTORY-1];
		newRight = (int32_t)TACHO_RightPosHistory[NOF_HISTORY-1];
	} else {
		newLeft = (int32_t)TACHO_LeftPosHistory[TACHO_PosHistory_Index-1];
		newRight = (int32_t)TACHO_RightPosHistory[TACHO_PosHistory_Index-1];
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
	speedLeft = (int32_t)(deltaLeft*1000U/(TACHO_SAMPLE_PERIOD_MS*(NOF_HISTORY-1)));
	if (negLeft) {
		speedLeft = -speedLeft;
	}
	speedRight = (int32_t)(deltaRight*1000U/(TACHO_SAMPLE_PERIOD_MS*(NOF_HISTORY-1)));
	if (negRight) {
		speedRight = -speedRight;
	}
	TACHO_currLeftSpeed = -speedLeft; /* store current speed in global variable */
	TACHO_currRightSpeed = -speedRight; /* store current speed in global variable */
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
