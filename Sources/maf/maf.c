/***********************************************************************************************//**
 * @file		rmaf.c
 * @ingroup		maf
 * @brief 		Moving AVerage Filter
 *
 * <This is a detailed description.>
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_maf_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "tacho.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "CS1.h"


/*======================================= >> #DEFINES << =========================================*/
#define MAF_CONDITIONAL_RETURN(condVar_, trueVal_, falseVal_) ( (TRUE == condVar_)?(trueVal_) : (falseVal_) )
#define NOF_HISTORY (16U+1U)
/**
 * Number of samples for speed calculation (>0):the more, the better, but the slower.
 */

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/

/*!< for better accuracy, we calculate the speed over some samples */
static volatile Q4CLeft_QuadCntrType MAF_LeftPosHistory[NOF_HISTORY], MAF_RightPosHistory[NOF_HISTORY];
/*!< position index in history */
static volatile uint8_t MAF_PosHistory_Index = 0;

static int32_t MAF_currLeftSpeed = 0, MAF_currRightSpeed = 0;

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
int32_t MAF_Get_Speed(bool isLeft_)
{
	return (int32_t)MAF_CONDITIONAL_RETURN(isLeft_, MAF_currLeftSpeed, MAF_currRightSpeed);
}

void MAF_UpdateRingBuffer(Q4CLeft_QuadCntrType leftVal_, Q4CLeft_QuadCntrType rightVal_)
{
	MAF_LeftPosHistory[MAF_PosHistory_Index]  = leftVal_;
	MAF_RightPosHistory[MAF_PosHistory_Index] = rightVal_;
	MAF_PosHistory_Index++;
	if(MAF_PosHistory_Index >= NOF_HISTORY)
	{
		MAF_PosHistory_Index = 0;
	}
}

void MAF_Init(void)
{
	uint8_t i = 0u;
	CS1_CriticalVariable();

	CS1_EnterCritical();
	MAF_currLeftSpeed    = 0;
	MAF_currRightSpeed   = 0;
	MAF_PosHistory_Index = 0;

	for(i = 0u; i < sizeof(MAF_LeftPosHistory)/sizeof(MAF_LeftPosHistory[0]); i++)
	{
		MAF_LeftPosHistory[i] = 0;
	}
	for(i = 0u; i < sizeof(MAF_RightPosHistory)/sizeof(MAF_RightPosHistory[0]); i++)
	{
		MAF_RightPosHistory[i] = 0;
	}
	CS1_ExitCritical();
}

void MAF_Deinit(void)
{
	uint8_t i = 0u;
	CS1_CriticalVariable();

	CS1_EnterCritical();
	MAF_currLeftSpeed    = 0;
	MAF_currRightSpeed   = 0;
	MAF_PosHistory_Index = 0;

	for(i = 0u; i < sizeof(MAF_LeftPosHistory)/sizeof(MAF_LeftPosHistory[0]); i++)
	{
		MAF_LeftPosHistory[i] = 0;
	}
	for(i = 0u; i < sizeof(MAF_RightPosHistory)/sizeof(MAF_RightPosHistory[0]); i++)
	{
		MAF_RightPosHistory[i] = 0;
	}
	CS1_ExitCritical();
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
	CS1_CriticalVariable();

	CS1_EnterCritical();
	oldLeft = (int32_t)MAF_LeftPosHistory[MAF_PosHistory_Index]; /* oldest left entry */
	oldRight = (int32_t)MAF_RightPosHistory[MAF_PosHistory_Index]; /* oldest right entry */
	if (MAF_PosHistory_Index==0) { /* get newest entry */
		newLeft  = (int32_t)MAF_LeftPosHistory[NOF_HISTORY-1];
		newRight = (int32_t)MAF_RightPosHistory[NOF_HISTORY-1];
	} else{
		newLeft  = (int32_t)MAF_LeftPosHistory[MAF_PosHistory_Index-1];
		newRight = (int32_t)MAF_RightPosHistory[MAF_PosHistory_Index-1];
	}
	CS1_ExitCritical();
	deltaLeft = oldLeft-newLeft; /* delta of oldest position and most recent one */
	/* use unsigned arithmetic */
	if (deltaLeft < 0) {
		deltaLeft = -deltaLeft;
		negLeft   = TRUE;
	} else {
		negLeft = FALSE;
	}
	deltaRight = oldRight-newRight; /* delta of oldest position and most recent one */
	/* use unsigned arithmetic */
	if (deltaRight < 0) {
		deltaRight = -deltaRight;
		negRight   = TRUE;
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
	MAF_currLeftSpeed  = -speedLeft;  /* store current speed in global variable */
	MAF_currRightSpeed = -speedRight; /* store current speed in global variable */
}

#ifdef MASTER_maf_C_
#undef MASTER_maf_C_
#endif /* !MASTER_maf_C_ */
