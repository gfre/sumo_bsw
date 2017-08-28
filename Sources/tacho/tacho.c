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
#include "tacho_cfg.h"
#include "tacho_api.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "CS1.h"




/*======================================= >> #DEFINES << =========================================*/
#define FILTER_TABLE_INDEX_INVALID (0xFF)



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum TACHO_ID_e
{
	 TACHO_ID_LEFT = 0
	,TACHO_ID_RIGHT
	,TACHO_ID_CNT
} TACHO_ID_t;

typedef struct TACHO_Data_s
{
	TACHO_FltrItm_t *pActFltr;
	uint8_t actFltrIdx;
	int16_t rawSpd[TACHO_ID_CNT];
	int16_t fltrdSpd[TACHO_ID_CNT];
	int32_t curPos[TACHO_ID_CNT];
	int32_t prevPos[TACHO_ID_CNT];
} TACHO_Data_t;



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static StdRtn_t Calc_RawSpd(TACHO_ID_t id_);
static StdRtn_t Set_TachoFltr(uint8_t idx_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static TACHO_Data_t data ={0};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static StdRtn_t Set_TachoFltr(uint8_t idx_)
{
	StdRtn_t retVal = ERR_OK;
	TACHO_FltrItmTbl_t *pFltrTbl = NULL;


	if( idx_ != data.actFltrIdx )
	{
		pFltrTbl = Get_pFltrTbl();
		if( (NULL != data.pActFltr) && (NULL != data.pActFltr->deinitFct) &&
			(NULL != pFltrTbl) && (NULL != pFltrTbl->aFltrs ) &&
			(NULL != &(pFltrTbl->aFltrs[idx_])) && (NULL != pFltrTbl->aFltrs[idx_].initFct))

		{
			/* De-initialise old filter type */
			data.pActFltr->deinitFct();
			if(TRUE == data.pActFltr->reqRawSpd)
			{
				data.rawSpd[TACHO_ID_LEFT]  = 0;
				data.rawSpd[TACHO_ID_RIGHT] = 0;
			}

			/* Initialise new filter tpye */
			data.pActFltr = &(pFltrTbl->aFltrs[idx_]);
			data.actFltrIdx = idx_;
			data.pActFltr->initFct();
		}
		else
		{
			retVal |= ERR_PARAM_ADDRESS;
		}
	}
	return retVal;
}



static StdRtn_t Calc_RawSpd(TACHO_ID_t id_)
{
	StdRtn_t retVal = ERR_PARAM_ID;
	int32_t deltaPos = 0;
	int16_t	rawSpeed = 0;
	bool negFlag = FALSE;

	if( (0 <= id_) && (TACHO_ID_CNT > id_) )
	{
		deltaPos = data.curPos[id_] - data.prevPos[id_];
		if(deltaPos < 0)
		{
			deltaPos = -deltaPos;
			negFlag   = TRUE;
		}
		rawSpeed =(int16_t)(deltaPos*1000U/(TACHO_SAMPLE_PERIOD_MS));

		if(TRUE == negFlag)
		{
			data.rawSpd[id_] = -rawSpeed;
		}
		else
		{
			data.rawSpd[id_]  = rawSpeed;
		}
		retVal = ERR_OK;
	}

	return retVal;
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void TACHO_Sample(void) {
	static int cnt = 0;

	/* get called from the RTOS tick counter. Divide the frequency. */
	cnt += portTICK_PERIOD_MS;
	if (cnt < TACHO_SAMPLE_PERIOD_MS) { /* sample only every TACHO_SAMPLE_PERIOD_MS */
		return;
	}
	cnt = 0; /* reset counter */

	CS1_CriticalVariable();

	CS1_EnterCritical();
	data.prevPos[TACHO_ID_LEFT]  = data.curPos[TACHO_ID_LEFT];
	data.prevPos[TACHO_ID_RIGHT] = data.curPos[TACHO_ID_RIGHT];

	data.curPos[TACHO_ID_LEFT]  = (int32_t)Q4CLeft_GetPos();
	data.curPos[TACHO_ID_RIGHT] = (int32_t)Q4CRight_GetPos();

	if( ( NULL != data.pActFltr) && ( NULL != data.pActFltr->sampleCbFct) )
	{
		data.pActFltr->sampleCbFct();
	}

	if(TRUE == data.pActFltr->reqRawSpd)
	{
		(void)Calc_RawSpd(TACHO_ID_LEFT);
		(void)Calc_RawSpd(TACHO_ID_RIGHT);
	}
	else
	{
		data.rawSpd[TACHO_ID_LEFT]  = TACHO_SPEED_VALUE_INVALID;
		data.rawSpd[TACHO_ID_RIGHT] = TACHO_SPEED_VALUE_INVALID;
	}
	CS1_ExitCritical();
}


void TACHO_Init(void)
{
	TACHO_FltrItmTbl_t *pFltrTbl = NULL;

	pFltrTbl = Get_pFltrTbl();

	if( (NULL != pFltrTbl) && (NULL != pFltrTbl->aFltrs ) &&  (TACHO_MAX_NUM_OF_FILTERS >= pFltrTbl->numFltrs) )
	{

		data.pActFltr = &(pFltrTbl->aFltrs[0]);
		data.actFltrIdx = 0;
		if( NULL != data.pActFltr->initFct)
		{
			data.pActFltr->initFct();
		}
	}
	else
	{
		/* error handling */
	}

}


void TACHO_Main(void)
{
	if( ( NULL != data.pActFltr ) && ( NULL != data.pActFltr->mainFct ) )
	{
		data.pActFltr->mainFct();
		data.fltrdSpd[TACHO_ID_LEFT]  = data.pActFltr->apiSpeedFct(TRUE);
		data.fltrdSpd[TACHO_ID_RIGHT] = data.pActFltr->apiSpeedFct(FALSE);
	}
	else
	{
		/* error handling */
	}
}


void TACHO_Deinit(void)
{
	if( ( NULL != data.pActFltr ) && ( NULL != data.pActFltr->deinitFct ) )
	{
		data.pActFltr->deinitFct();
	}
	data.pActFltr = NULL;
}


StdRtn_t TACHO_Set_FltrReq(uint8_t idx_)
{
	return Set_TachoFltr(idx_);
}

StdRtn_t TACHO_Read_PosLft(int32_t* pos_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != pos_)
	{
		*pos_  = data.curPos[TACHO_ID_LEFT];
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_PosRght(int32_t* pos_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != pos_)
	{
		*pos_  = data.curPos[TACHO_ID_RIGHT];
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_RawSpdLft(int16_t* spd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != spd_)
	{
		*spd_  = data.rawSpd[TACHO_ID_LEFT];
		retVal   = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_RawSpdRght(int16_t* spd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != spd_)
	{
		*spd_  = data.rawSpd[TACHO_ID_RIGHT];
		retVal   = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_SpdLft(int16_t* spd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != spd_)
	{
		*spd_ = data.fltrdSpd[TACHO_ID_LEFT];
		retVal = ERR_OK;
	}
	return retVal;
}

StdRtn_t TACHO_Read_SpdRght(int16_t* spd_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != spd_)
	{
		*spd_ = data.fltrdSpd[TACHO_ID_RIGHT];
		retVal  = ERR_OK;
	}
	return retVal;
}

uint8_t TACHO_Get_ActFltrIdx(void)
{
	return data.actFltrIdx;
}

#ifdef MASTER_tacho_C_
#undef MASTER_tacho_C_
#endif /* !MASTER_tacho_C_ */
