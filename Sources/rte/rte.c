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


StdRetType RTE_Write_LedRiOn()
{
	LED1_On();
	return RET_OK;
}

StdRetType RTE_Write_LedRiOff()
{
	LED1_Off();
	return RET_OK;
}

StdRetType RTE_Write_LedRiNeg()
{
	LED1_Neg();
	return RET_OK;
}



StdRetType RTE_Write_LedRiSt(uint8_t state)
{
	if(0u==state)
	{
		LED1_Put(FALSE);
	}
	else
	{
		LED1_Put(TRUE);
	}
	return RET_OK;
}


StdRetType RTE_Read_LedRiSt(uint8_t *state)
{
	uint8_t retVal = RET_INVALID;
	if(NULL!=state)
	{
		*state = (uint8_t)LED1_Get();
		retVal = RET_OK;
	}
	return retVal;
}



StdRetType RTE_Write_LedLeOn()
{
	LED2_On();
	return RET_OK;
}

StdRetType RTE_Write_LedLeOff()
{
	LED2_Off();
	return RET_OK;
}

StdRetType RTE_Write_LedLeNeg()
{
	LED2_Neg();
	return RET_OK;
}



StdRetType RTE_Write_LedLeSt(uint8_t state)
{
	if(0u==state)
	{
		LED2_Put(FALSE);
	}
	else
	{
		LED2_Put(TRUE);
	}
	return RET_OK;
}


StdRetType RTE_Read_LedLeSt(uint8_t *state)
{
	uint8_t retVal = RET_INVALID;
	if(NULL!=state)
	{
		*state = (uint8_t)LED2_Get();
		retVal = RET_OK;
	}
	return retVal;
}



#ifdef MASTER_ID_C_
#undef MASTER_ID_C_
#endif
