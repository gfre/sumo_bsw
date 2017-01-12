/*******************************************************************************
 * @brief 	This is the interface entrance layer for students.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date	 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#define MASTER_ID_C_
#include "LED1.h"
#include "LED2.h"
#include "rte.h"


StdRtnType RTE_Write_LedRiOn()
{
	LED1_On();
	return RTN_OK;
}

StdRtnType RTE_Write_LedRiOff()
{
	LED1_Off();
	return RTN_OK;
}

StdRtnType RTE_Write_LedRiNeg()
{
	LED1_Neg();
	return RTN_OK;
}



StdRtnType RTE_Write_LedRiSt(uint8_t state)
{
	if(0u==state)
	{
		LED1_Put(FALSE);
	}
	else
	{
		LED1_Put(TRUE);
	}
	return RTN_OK;
}


StdRtnType RTE_Read_LedRiSt(uint8 *state_)
{
	uint8 retVal = RTN_INVALID;
	if(NULL!=state_)
	{
		*state_ = (uint8)LED1_Get();
		retVal = RTN_OK;
	}
	return retVal;
}



StdRtnType RTE_Write_LedLeOn()
{
	LED2_On();
	return RTN_OK;
}

StdRtnType RTE_Write_LedLeOff()
{
	LED2_Off();
	return RTN_OK;
}

StdRtnType RTE_Write_LedLeNeg()
{
	LED2_Neg();
	return RTN_OK;
}



StdRtnType RTE_Write_LedLeSt(uint8 state_)
{
	if(0u==state_)
	{
		LED2_Put(FALSE);
	}
	else
	{
		LED2_Put(TRUE);
	}
	return RTN_OK;
}


StdRtnType RTE_Read_LedLeSt(uint8 *state_)
{
	uint8 retVal = RTN_INVALID;
	if(NULL!=state_)
	{
		*state_ = (uint8)LED2_Get();
		retVal = RTN_OK;
	}
	return retVal;
}



#ifdef MASTER_ID_C_
#undef MASTER_ID_C_
#endif
