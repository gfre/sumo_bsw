/***********************************************************************************************//**
 * @file		refl.c
 * @ingroup		refl
 * @brief 		Implements driver software for reflectance sensor array
 *
 * <This is a detailed description.>
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_refl_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"
#include "refl.h"
#include "refl_cfg.h"
#include "refl_api.h"
#include "buz_api.h"
#include "nvm_api.h"
#include "sh_api.h"

#include "LED_IR.h"  /*Infrared LED's ON/OFF*/
#include "WAIT1.h"
#include "RefCnt.h" /* timer counter to measure reflectance */
#include "IR1.h" /* Infrared LED Sensors*/
#include "IR2.h"
#include "IR3.h"
#include "IR4.h"
#include "IR5.h"
#include "IR6.h"
#include "TMOUT1.h"
#include "FRTOS1.h"



/*======================================= >> #DEFINES << =========================================*/
#if defined( CAU_SUMO_PLT_NUM_OF_REFL_SENSORS )
	#if CAU_SUMO_PLT_NUM_OF_REFL_SENSORS
	/**
	 * Number of reflectance sensors
	 */
	#define NUM_OF_REFL_SENSORS		CAU_SUMO_PLT_NUM_OF_REFL_SENSORS
	#else
	#undef CAU_SUMO_PLT_NUM_OF_REFL_SENSORS
	#endif
#endif

#ifndef NUM_OF_REFL_SENSORS
#error Unknown number of reflectance sensors
#endif

/**
 *
 */
#define REFL_MIDDLE_LINE_VALUE  ((NUM_OF_REFL_SENSORS+1)*1000/2)

/**
 * Maximum value for REFL_GetLine()
 */
#define REFL_MAX_LINE_VALUE     ( (NUM_OF_REFL_SENSORS + 1) * 1000 )

/**
 * 1/4 of full sensor values
 */
#define MIN_LEFT_RIGHT_SUM      ( (NUM_OF_REFL_SENSORS * 1000) / 4 )

/**
 *
 */
#define MAX_SENSOR_VALUE  			((REFL_SnsrTime_t)-1)

/**
 *
 */
#define RETURN_LAST_VALUE			0


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct REFL_SnsrData_s {
	REFL_SnsrTime_t raw[NUM_OF_REFL_SENSORS];
	REFL_SnsrTime_t norm[NUM_OF_REFL_SENSORS];
} REFL_SnsrData_t;


typedef struct REFL_CfgData_s
{
	bool swcEnabled;
	bool irLedEnabled;
	const REFL_Cfg_t *pCfg;
	NVM_ReflCalibData_t calibData;
	REFL_SnsrTime_t maxValidRawVal;
}REFL_CfgData_t;

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void S1_SetOutput(void);
static void S1_SetInput(void);
static void S1_SetVal(void);
static bool S1_GetVal(void);

static void S2_SetOutput(void);
static void S2_SetInput(void);
static void S2_SetVal(void);
static bool S2_GetVal(void);

static void S3_SetOutput(void);
static void S3_SetInput(void);
static void S3_SetVal(void);
static bool S3_GetVal(void);

static void S4_SetOutput(void);
static void S4_SetInput(void);
static void S4_SetVal(void);
static bool S4_GetVal(void);

static void S5_SetOutput(void);
static void S5_SetInput(void);
static void S5_SetVal(void);
static bool S5_GetVal(void);

static void S6_SetOutput(void);
static void S6_SetInput(void);
static void S6_SetVal(void);
static bool S6_GetVal(void);

static StdRtn_t MeasureSnsrRawData(REFL_SnsrTime_t *aRawData_, uint8_t cntOfSnsrs_, RefCnt_TValueType timeoutCntVal);
static StdRtn_t CalibMinMaxVal(REFL_SnsrTime_t *rawData_, REFL_SnsrTime_t *minData_, REFL_SnsrTime_t *maxData_, uint8_t cntOfSnsrs_);
static StdRtn_t CalcNormData(REFL_SnsrTime_t *normData_, REFL_SnsrTime_t *rawData_);
static REFL_LineKind_t CalcLineKind(REFL_SnsrTime_t *normData_, uint8_t cntOfSnsrs_);
static uint16_t CalcLineCenter(const REFL_SnsrTime_t *aNormData_, uint8_t cntOfSnsrs_, REFL_LineBW_t lineBW_);
static uint16_t CalcLineWidth(const REFL_SnsrTime_t *aNormData_, uint8_t cntOfSnsrs_);
static void RunLineDetection(void);
static void ProcStateMachine(void);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static volatile REFL_State_t reflState = REFL_STATE_INIT;

static LDD_TDeviceData *pTmrHdl;
static xSemaphoreHandle pBinSemphrHdl = NULL;

static REFL_Line_t dctdLine = {0, REFL_LINE_NONE, 0u};
static REFL_SnsrData_t snsrData ={0};
static REFL_CfgData_t cfgData = {0};

static const SnsrIOFcts_t snsrIOFcts[NUM_OF_REFL_SENSORS] = {
  {S1_SetOutput, S1_SetInput, S1_SetVal, S1_GetVal},
  {S2_SetOutput, S2_SetInput, S2_SetVal, S2_GetVal},
  {S3_SetOutput, S3_SetInput, S3_SetVal, S3_GetVal},
  {S4_SetOutput, S4_SetInput, S4_SetVal, S4_GetVal},
  {S5_SetOutput, S5_SetInput, S5_SetVal, S5_GetVal},
  {S6_SetOutput, S6_SetInput, S6_SetVal, S6_GetVal},
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void S1_SetOutput(void) { IR1_SetOutput(); }
static void S1_SetInput(void) { IR1_SetInput(); }
static void S1_SetVal(void) { IR1_SetVal(); }
static bool S1_GetVal(void) { return IR1_GetVal(); }

static void S2_SetOutput(void) { IR2_SetOutput(); }
static void S2_SetInput(void) { IR2_SetInput(); }
static void S2_SetVal(void) { IR2_SetVal(); }
static bool S2_GetVal(void) { return IR2_GetVal(); }

static void S3_SetOutput(void) { IR3_SetOutput(); }
static void S3_SetInput(void) { IR3_SetInput(); }
static void S3_SetVal(void) { IR3_SetVal(); }
static bool S3_GetVal(void) { return IR3_GetVal(); }

static void S4_SetOutput(void) { IR4_SetOutput(); }
static void S4_SetInput(void) { IR4_SetInput(); }
static void S4_SetVal(void) { IR4_SetVal(); }
static bool S4_GetVal(void) { return IR4_GetVal(); }

static void S5_SetOutput(void) { IR5_SetOutput(); }
static void S5_SetInput(void) { IR5_SetInput(); }
static void S5_SetVal(void) { IR5_SetVal(); }
static bool S5_GetVal(void) { return IR5_GetVal(); }

static void S6_SetOutput(void) { IR6_SetOutput(); }
static void S6_SetInput(void) { IR6_SetInput(); }
static void S6_SetVal(void) { IR6_SetVal(); }
static bool S6_GetVal(void) { return IR6_GetVal(); }

static void IR_on(bool on) {
  if (on) {
    LED_IR_On();
  } else {
    LED_IR_Off();
  }
}

/*!
 * \brief Measures the time until the sensor discharges
 * \param raw Array to store the raw values.
 * \return ERR_OVERFLOW if there is a timeout, ERR_OK otherwise
 */
static StdRtn_t MeasureSnsrRawData(REFL_SnsrTime_t *aRawData_, uint8_t cntOfSnsrs_, RefCnt_TValueType timeoutTmrVal_)
{
	StdRtn_t retVal = ERR_OK;
	uint8_t i = 0u;
	uint8_t chkSum = 0u;
	bool aMeasured[cntOfSnsrs_];
	RefCnt_TValueType tmrVal = 0u;

	if (TRUE == cfgData.swcEnabled)
	{
		if ( ( NULL != aRawData_ ) && (NUM_OF_REFL_SENSORS >= cntOfSnsrs_) )
		{
			if (TRUE == cfgData.irLedEnabled)
			{
				/* IR LED's on */
				LED_IR_On();
				WAIT1_Waitus(200);
			}

			for(i = 0u; i < cntOfSnsrs_; i++)
			{
				/* set I/O array as output */
				snsrIOFcts[i].SetOutput();
				/* put high */
				snsrIOFcts[i].SetVal();
				/* reset measured flag */
				aMeasured[i] = FALSE;
			}
			/* give at least 10 us to charge the capacitor */
			WAIT1_Waitus(50);

			taskENTER_CRITICAL();
			for(i = 0u; i < cntOfSnsrs_; i++)
			{
				/* set I/O array as input */
				snsrIOFcts[i].SetInput();
			}
			/* reset timer counter */
			(void)RefCnt_ResetCounter(pTmrHdl);

			/* do measurement */
			while ( (chkSum != NUM_OF_REFL_SENSORS) && (tmrVal <= timeoutTmrVal_) )
			{
				chkSum = 0u;
				for (i = 0u; i < cntOfSnsrs_; i++)
				{
					/* not measured yet? */
					if (FALSE == aMeasured[i] && snsrIOFcts[i].GetVal() == FALSE)
					{
						aRawData_[i] = tmrVal;
						aMeasured[i] = TRUE;
					}
					chkSum += aMeasured[i];
				}
				tmrVal = RefCnt_GetCounterValue(pTmrHdl);

			}

			taskEXIT_CRITICAL();

			LED_IR_Off();

			/* not all sensors received measurement --> timeout detected ? */
			if (chkSum != NUM_OF_REFL_SENSORS)
			{
				for( i = 0u; (i < cntOfSnsrs_) && (FALSE == aMeasured[i]); i++ )
				{
					aRawData_[i] = (cfgData.calibData.maxVal[i] != 0) ? (cfgData.calibData.maxVal[i]) : (timeoutTmrVal_);
				}
			}
		}
		else
		{
			retVal = ERR_PARAM_ADDRESS;
		}
	}
	else
	{
		retVal = ERR_DISABLED;
	}
	return retVal;
}

static StdRtn_t CalibMinMaxVal(REFL_SnsrTime_t *rawData_, REFL_SnsrTime_t *minData_, REFL_SnsrTime_t *maxData_, uint8_t cntOfSnsrs_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	uint8_t i = 0u;

	if ( ( NULL != minData_ ) && ( NULL != maxData_ ) && ( NULL != rawData_ ) && ( NUM_OF_REFL_SENSORS >= cntOfSnsrs_ ) )
	{
		retVal = MeasureSnsrRawData( rawData_, cntOfSnsrs_, REFL_TIMEOUT_US_TO_TICKS(cfgData.pCfg->measTimeOutUS) );
		if ( ERR_OK == retVal )
		{	  /* if timeout, do not count values */
			for( i = 0u; i < cntOfSnsrs_; i++ )
			{
				if (rawData_[i] < minData_[i])
				{
					minData_[i] = rawData_[i];
				}
				if (rawData_[i]> maxData_[i])
				{
					maxData_[i] = rawData_[i];
				}
			}
		}
	}
	return retVal;
}

static StdRtn_t CalcNormData(REFL_SnsrTime_t *normData_, REFL_SnsrTime_t *rawData_)
{
	StdRtn_t retVal = ERR_OK;
	uint8_t i = 0u;
	int32_t x = 0, denominator = 0;


	/* no calibration data? */
	if (cfgData.calibData.maxVal[0] == 0)
	{
		return ERR_PARAM_DATA;
	}

	for(i = 0u; i < NUM_OF_REFL_SENSORS; i++)
	{
		x = 0;
		denominator = cfgData.calibData.maxVal[i] - cfgData.calibData.minVal[i];
		if (0 != denominator)
		{
		  x = ( ( (int32_t)rawData_[i] - cfgData.calibData.minVal[i] ) * 1000 ) / denominator;
		}

		if (x < 0)
		{
		  x = 0;
		}
		else if (x > 1000)
		{
		  x = 1000;
		}
		normData_[i] = x;
	}

	return retVal;
}

/*
 * Operates the same as read calibrated, but also returns an
 * estimated position of the robot with respect to a dctdLine. The
 * estimate is made using a weighted average of the sensor indices
 * multiplied by 1000, so that a return value of 1000 indicates that
 * the dctdLine is directly below sensor 0, a return value of 2000
 * indicates that the dctdLine is directly below sensor 1, 2000
 * indicates that it's below sensor 2000, etc. Intermediate
 * values indicate that the dctdLine is between two sensors. The
 * formula is:
 *
 * 1000*value0 + 2000*value1 + 3000*value2 + ...
 * --------------------------------------------
 * value0 + value1 + value2 + ...
 *
 * This function distinguishes between a dark dctdLine (high values)
 * surrounded by white (low values) and a bright dctdLine surrounded by black.
 */

static uint16_t CalcLineCenter(const REFL_SnsrTime_t *aNormData_, uint8_t cntOfSnsrs_, REFL_LineBW_t lineBW_)
{
  uint8_t i = 0u;
  uint32_t avg = 0u; /* this is for the weighted total, which is long */
  uint16_t sum = 0u; /* this is for the denominator which is <= 64000 */
  uint16_t mul = 0u; /* multiplication factor, 0, 1000, 2000, 3000 ... */
  uint16_t normVal = 0;

  avg = 0u;
  sum = 0u;
  mul = 1000u;
  for(i = 0u; i < cntOfSnsrs_; i++)
  {
	  normVal = MIN(1000u, aNormData_[i]);
	  normVal = (REFL_LINE_WHITE == lineBW_) ? (1000u - normVal) : (normVal);

	  /* only average in values that are above a noise threshold */
	  if(normVal > cfgData.pCfg->minNoiseVal)
	  {
		  avg += (uint32_t)( (uint32_t)normVal * (uint32_t)mul);
		  sum += normVal;
	  }
	  mul += 1000u;
  }

  if (sum > 0u)
  {
	  avg /= sum;
  }

  return avg;
}

static bool SensorsSaturated(void) {
#if 0
  int i, cnt;

  /* check if robot is in the air or does see something */
  cnt = 0;
  for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
    if (SensorRaw[i]==MAX_SENSOR_VALUE) { /* sensor not seeing anything? */
      cnt++;
    }
  }
  return (cnt==NUM_OF_REFL_SENSORS); /* all sensors see raw max value: not on the ground? */
#else
  return FALSE;
#endif
}

static REFL_LineKind_t CalcLineKind(REFL_SnsrTime_t *normData_, uint8_t cntOfSnsrs_)
{
	uint32_t sum = 0u, sumLeft = 0u, sumRight = 0u, outerLeft = 0u, outerRight = 0u;
	uint8_t i = 0u;
	REFL_LineKind_t lineKind = REFL_LINE_NONE;
	bool fullLine = TRUE;

	if( ( NULL !=  normData_) && ( NUM_OF_REFL_SENSORS >= cntOfSnsrs_) )
	{
		/* check if robot is in the air or does see something */
		if (SensorsSaturated())
		{
		  lineKind = REFL_LINE_AIR;
		}
		else
		{
			/* check the dctdLine type */
			sum = 0u;
			sumLeft  = 0u;
			sumRight = 0u;
			outerLeft = normData_[0];
			outerRight = normData_[cntOfSnsrs_-1];
			for( i = 0u ; i < cntOfSnsrs_; i++ )
			{
				if (normData_[i] >= cfgData.pCfg->minLineVal)
				{ /* count only dctdLine values */
				  sum += normData_[i];
				  if ( i <  cntOfSnsrs_ / 2u )
				  {
					  sumLeft += normData_[i];
				  }
				  else
				  {
					  sumRight += normData_[i];
				  }
				}
				else
				{
					fullLine = FALSE;
				}
			}

			if (TRUE == fullLine)
			{
				lineKind = REFL_LINE_FULL;
			}
			else if ( outerLeft >= cfgData.pCfg->minLineVal && outerRight < cfgData.pCfg->minLineVal && sumLeft>MIN_LEFT_RIGHT_SUM && sumRight<MIN_LEFT_RIGHT_SUM)
			{
				lineKind = REFL_LINE_LEFT; /* dctdLine going to the left side */
			}
			else if (outerLeft < cfgData.pCfg->minLineVal && outerRight >= cfgData.pCfg->minLineVal && sumRight>MIN_LEFT_RIGHT_SUM && sumLeft<MIN_LEFT_RIGHT_SUM)
			{
				lineKind = REFL_LINE_RIGHT; /* dctdLine going to the right side */
			}
			else if (outerLeft >= cfgData.pCfg->minLineVal && outerRight >= cfgData.pCfg->minLineVal && sumRight>MIN_LEFT_RIGHT_SUM && sumLeft>MIN_LEFT_RIGHT_SUM)
			{
				lineKind = REFL_LINE_FULL; /* full dctdLine */
			}
			else if (sumRight==0 && sumLeft==0 && sum == 0) {
				lineKind = REFL_LINE_NONE; /* no dctdLine */
			}
			else
			{
				lineKind = REFL_LINE_STRAIGHT; /* straight dctdLine forward */
			}
		}
	}	return lineKind;
}

static uint16_t CalcLineWidth(const REFL_SnsrTime_t *normData_, uint8_t cntOfSnsrs_)
{
	uint32_t lineWidth = 0u;
	uint8_t i = 0u;

	for(i = 0u; i < cntOfSnsrs_; i++)
	{
	  if (normData_[i] >= cfgData.pCfg->minNoiseVal) /* sensor not seeing anything? */
	  {
		  lineWidth += (uint32_t)normData_[i];
	  }
	}

	return (uint16_t)lineWidth;
}

static void RunLineDetection(void)
{
	dctdLine.center = 0xFFFFu;
	dctdLine.width = 0xFFFFu;
	dctdLine.kind = REFL_LINE_NONE;

	if(ERR_OK == MeasureSnsrRawData(snsrData.raw, NUM_OF_REFL_SENSORS, cfgData.maxValidRawVal))
	{
		if( ERR_OK == CalcNormData(snsrData.norm, snsrData.raw) )
		{
			dctdLine.center = CalcLineCenter(snsrData.norm, NUM_OF_REFL_SENSORS, cfgData.pCfg->lineBW);
			dctdLine.width  = CalcLineWidth(snsrData.norm, NUM_OF_REFL_SENSORS);
			dctdLine.kind   = CalcLineKind(snsrData.norm, NUM_OF_REFL_SENSORS);
		}
	}
}


static void ProcStateMachine(void)
{
  uint8_t i = 0u;
  static NVM_ReflCalibData_t *pCalibMinMaxDataTmp = NULL;

  switch (reflState)
  {
  default:
  case REFL_STATE_INIT:
	  if (NVM_Read_ReflCalibData(&cfgData.calibData) == ERR_OK)  /* use calibration data from FLASH */
      {
		  for(i = 0u; i < NUM_OF_REFL_SENSORS; i++)
		  {
			  if (cfgData.calibData.maxVal[i] > cfgData.maxValidRawVal)
			  {
				  cfgData.maxValidRawVal = cfgData.calibData.maxVal[i];
			  }
		  }

		  /* limit to timeout value */
		  if (cfgData.maxValidRawVal > REFL_TIMEOUT_US_TO_TICKS(cfgData.pCfg->measTimeOutUS))
		  {
			  cfgData.maxValidRawVal = REFL_TIMEOUT_US_TO_TICKS(cfgData.pCfg->measTimeOutUS);
		  }

		  reflState = REFL_STATE_READY;
      }
      else
      {
    	  SH_SENDSTR((unsigned char*)"no calibration data present.\r\n");
    	  reflState = REFL_STATE_NOT_CALIBRATED;
      }
      break;

  case REFL_STATE_READY:
	  RunLineDetection();
	  if (FRTOS1_xSemaphoreTake(pBinSemphrHdl, 0)==pdTRUE)
	  {
		  reflState = REFL_STATE_START_CALIBRATION;
	  }
	  break;

    case REFL_STATE_NOT_CALIBRATED:
    	FRTOS1_vTaskDelay(80/portTICK_PERIOD_MS); /* no need to sample that fast: this gives 80+20=100 ms */
		(void)MeasureSnsrRawData(snsrData.raw, NUM_OF_REFL_SENSORS, REFL_TIMEOUT_US_TO_TICKS(cfgData.pCfg->measTimeOutUS));

		if (FRTOS1_xSemaphoreTake(pBinSemphrHdl, 0)==pdTRUE)
		{
		reflState = REFL_STATE_START_CALIBRATION;
		}

		break;

    case REFL_STATE_START_CALIBRATION:
    	if (pCalibMinMaxDataTmp != NULL)
		{
		reflState = REFL_STATE_INIT; /* error case */
		break;
		}
		pCalibMinMaxDataTmp = FRTOS1_pvPortMalloc(sizeof(NVM_ReflCalibData_t));
		if (pCalibMinMaxDataTmp != NULL)  /* success */
		{
		  for(i = 0u; i < NUM_OF_REFL_SENSORS; i++)
		  {
			  pCalibMinMaxDataTmp->minVal[i] = MAX_SENSOR_VALUE;
			  pCalibMinMaxDataTmp->maxVal[i] = 0;
			  snsrData.norm[i] = 0;
		  }
		  reflState = REFL_STATE_CALIBRATING;
		}
		else
		{
		  reflState = REFL_STATE_INIT; /* error case */
		}
		break;

    case REFL_STATE_CALIBRATING:
      FRTOS1_vTaskDelay(80/portTICK_PERIOD_MS); /* no need to sample that fast: this gives 80+20=100 ms */
      if (pCalibMinMaxDataTmp != NULL) /* safety check */
      {
    	  CalibMinMaxVal(snsrData.raw, pCalibMinMaxDataTmp->minVal, pCalibMinMaxDataTmp->maxVal, NUM_OF_REFL_SENSORS);
      }
      else
      {
    	  reflState = REFL_STATE_INIT; /* error case */
    	  break;
      }
      (void)BUZ_Beep(100, 50);


      if (FRTOS1_xSemaphoreTake(pBinSemphrHdl, 0)==pdTRUE)
      {
        reflState = REFL_STATE_STOP_CALIBRATION;
      }
      break;

    case REFL_STATE_STOP_CALIBRATION:
      reflState = REFL_STATE_SAVE_CALIBRATION;
      break;

    case REFL_STATE_SAVE_CALIBRATION:
      if(NVM_Save_ReflCalibData(pCalibMinMaxDataTmp) != ERR_OK)
      {
    	  SH_SENDSTR((unsigned char*)"Failed to save calib data.\r\n");
      }
      /* free memory */
      FRTOS1_vPortFree(pCalibMinMaxDataTmp);
      pCalibMinMaxDataTmp = NULL;
      if(NVM_Read_ReflCalibData(&cfgData.calibData) == ERR_OK)
      {
    	  SH_SENDSTR((unsigned char*)"Calibration data saved.\r\n");

    	  reflState = REFL_STATE_READY;
      }else
      {
    	  reflState = REFL_STATE_INIT;
      }
      break;
  } /* switch */
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
StdRtn_t REFL_Read_ReflCfg(REFL_Cfg_t *pCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != pCfg_)
	{
		if( NULL != cfgData.pCfg)
		{
			*pCfg_ = *(cfgData.pCfg);
			retVal = ERR_OK;
		}
		else
		{
			retVal = ERR_PARAM_DATA;
		}
	}
	return retVal;
}

StdRtn_t REFL_Read_DctdLine(REFL_Line_t *dctdLine_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;

	if( NULL != dctdLine_)
	{
		*dctdLine_ = dctdLine;
		retVal = ERR_OK;
	}
	return retVal;
}

uint16_t REFL_Get_DctdLineCenter(bool *onLine) {
  *onLine = (dctdLine.center > 0) && (dctdLine.center < REFL_MAX_LINE_VALUE);
  return dctdLine.center;
}

REFL_LineKind_t REFL_Get_DctdLineKind(void)			{ return dctdLine.kind; }

uint16_t REFL_Get_DctdLineWidth(void)		 		{ return dctdLine.width; }

uint8_t REFL_Get_NumOfSnsrs(void)					{ return NUM_OF_REFL_SENSORS; }

REFL_State_t REFL_Get_State(void)					{ return reflState; }

NVM_ReflCalibData_t* REFL_Get_pCalibData(void)		{ return &cfgData.calibData; }

bool REFL_Get_SwcEnbldSt(void)						{ return cfgData.swcEnabled; }

void REFL_Set_SwcEnbldSt(bool state_)				{ cfgData.swcEnabled = (state_ & TRUE); }

bool REFL_Get_IrLedSt(void)							{ return cfgData.irLedEnabled;}

void REFL_Set_IrLedSt(bool state_)					{ cfgData.irLedEnabled = (state_ & TRUE); }

REFL_SnsrTime_t REFL_Get_RawSnsrVal(uint8_t idx_)	{ return snsrData.raw[idx_]; }

REFL_SnsrTime_t REFL_Get_NormSnsrVal(uint8_t idx_)	{ return snsrData.norm[idx_]; }

bool REFL_CanUseSensor(void) 						{ return reflState==REFL_STATE_READY; }

void REFL_Give_Smphr4CalibStartStop(void)
{
	cfgData.swcEnabled = TRUE;
	if (reflState==REFL_STATE_NOT_CALIBRATED || reflState==REFL_STATE_CALIBRATING || reflState==REFL_STATE_READY)
	{
		(void)xSemaphoreGive(pBinSemphrHdl);
	}
}

void REFL_Init(void)
{
	cfgData.pCfg = Get_pReflCfg();

	if (cfgData.pCfg != NULL )
	{
		pTmrHdl = RefCnt_Init(NULL);

		reflState = REFL_STATE_INIT;

		/* Init detected line */
		dctdLine.kind = REFL_LINE_NONE;
		dctdLine.center = 0;
		dctdLine.width = 0u;

		cfgData.irLedEnabled = TRUE; /* IR LED's on */
		cfgData.swcEnabled = TRUE;


		FRTOS1_vSemaphoreCreateBinary(pBinSemphrHdl);
		if (pBinSemphrHdl == NULL) /* semaphore creation failed */
		{
			for(;;){} /* error */
		}
		(void)FRTOS1_xSemaphoreTake(pBinSemphrHdl, 0); /* empty token */
		FRTOS1_vQueueAddToRegistry(pBinSemphrHdl, "REFL_BinSemphr_StartStop");
	}
	else
	{
	    for(;;); /* error case */
	}
}

void REFL_MainFct(void) {
    ProcStateMachine();
}



#ifdef MASTER_refl_C_
#undef MASTER_refl_C_
#endif /* !MASTER_refl_C_ */
