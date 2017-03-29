/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	27.03.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file ind.c
 * 
 *==================================================================================================
 */

#define MASTER_ind_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "ind.h"
#include "ind_Types.h"
#include "LED1.h"
#include "LED2.h"


/*======================================= >> #DEFINES << =========================================*/
#define LED1_TIMER_NAME 			("LED1_Timer")
#define LED2_TIMER_NAME 			("LED2_Timer")
#define LEDx_TIMER_TICKS_TO_WAIT 	(0UL)

#define LED1_TIMER 					(timer[0u])
#define LED2_TIMER 					(timer[1u])



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void Ind1TimerCbFct(TimerHandle_t timer_);
static void Ind2TimerCbFct(TimerHandle_t timer_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static TimerHandle_t timer[2u] = {NULL, NULL};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void Ind1TimerCbFct(TimerHandle_t timer_)
{
	if ( NULL != timer_)
	{
		LED1_Neg();
	}

	return;
}

static void Ind2TimerCbFct(TimerHandle_t timer_)
{
	if ( NULL != timer_)
	{
		LED2_Neg();
	}

	return;
}




/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t IND_Flash_LED1WithPerMS(const uint16_t perMS_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != LED1_TIMER)
	{
		LED1_Off();
		FRTOS1_xTimerChangePeriod(LED1_TIMER, pdMS_TO_TICKS( perMS_/2u ), LEDx_TIMER_TICKS_TO_WAIT);
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t IND_Flash_LED2WithPerMS(const uint16_t perMS_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != LED2_TIMER)
	{
		LED2_Off();
		FRTOS1_xTimerChangePeriod(LED2_TIMER, pdMS_TO_TICKS( perMS_/2u ), LEDx_TIMER_TICKS_TO_WAIT);
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t IND_Set_LED1On()
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != LED1_TIMER)
	{
		FRTOS1_xTimerStop(LED1_TIMER, LEDx_TIMER_TICKS_TO_WAIT);
		LED1_On();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t IND_Set_LED2On()
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != LED2_TIMER)
	{
		FRTOS1_xTimerStop(LED2_TIMER, LEDx_TIMER_TICKS_TO_WAIT);
		LED2_On();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t IND_Set_LED1Off()
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != LED1_TIMER)
	{
		FRTOS1_xTimerStop(LED1_TIMER, LEDx_TIMER_TICKS_TO_WAIT);
		LED1_Off();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t IND_Set_LED2Off()
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != LED2_TIMER)
	{
		FRTOS1_xTimerStop(LED2_TIMER, LEDx_TIMER_TICKS_TO_WAIT);
		LED2_Off();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t IND_Set_LED1Toggle()
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != LED1_TIMER)
	{
		FRTOS1_xTimerStop(LED1_TIMER, LEDx_TIMER_TICKS_TO_WAIT);
		LED1_Neg();
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t IND_Set_LED2Toggle()
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != LED2_TIMER)
	{
		FRTOS1_xTimerStop(LED2_TIMER, LEDx_TIMER_TICKS_TO_WAIT);
		LED2_Neg();
		retVal = ERR_OK;
	}
	return retVal;
}

void IND_Init(void)
{
	/* Create the LED1 timer, storing the handle to the created timer in xOneShotTimer. */
	LED1_TIMER = FRTOS1_xTimerCreate(
			/* Text name for the software timer - not used by FreeRTOS. */
			LED1_TIMER_NAME,
			/* The software timer's period in ticks. */
			pdMS_TO_TICKS( 1000u ),
			/* Setting uxAutoRealod */
			pdTRUE,
			/* This example does not use the timer id. */
			NULL,
			/* The callback function to be used by the software timer being created. */
			Ind1TimerCbFct );
	LED2_TIMER = FRTOS1_xTimerCreate(
			LED2_TIMER_NAME,
			pdMS_TO_TICKS( 1000u ),
			pdTRUE,
			NULL,
			Ind2TimerCbFct );

	if( (NULL == LED1_TIMER) || ( NULL == LED2_TIMER) )
	{
		for(;;);
	}
}


void IND_Main(void)
{
	/* nothing to do */
}


#ifdef MASTER_ind_C_
#undef MASTER_ind_C_
#endif /* !MASTER_ind_C_ */
