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
 * Start/Stop calibration commands
 */
#define REFL_START_STOP_CALIB      	1

/**
 *
 */
#define MAX_SENSOR_VALUE  			((SensorTimeType)-1)

/**
 *
 */
#define RETURN_LAST_VALUE			0


/*=================================== >> TYPE DEFINITIONS << =====================================*/




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

static void IR_on(bool on);

static bool REFL_MeasureRaw(SensorTimeType raw[NUM_OF_REFL_SENSORS], RefCnt_TValueType timeoutCntVal);

static void REFL_CalibrateMinMax(SensorTimeType min[NUM_OF_REFL_SENSORS], SensorTimeType max[NUM_OF_REFL_SENSORS], SensorTimeType raw[NUM_OF_REFL_SENSORS]);

static void ReadCalibrated(SensorTimeType calib[NUM_OF_REFL_SENSORS], SensorTimeType raw[NUM_OF_REFL_SENSORS]);

static int ReadLine(SensorTimeType calib[NUM_OF_REFL_SENSORS], SensorTimeType raw[NUM_OF_REFL_SENSORS], Refl_LineBW_t lineBW_);

static bool SensorsSaturated(void);

static REFL_LineKind ReadLineKind(SensorTimeType val[NUM_OF_REFL_SENSORS]);

static uint16_t CalculateReflLineWidth(SensorTimeType calib[NUM_OF_REFL_SENSORS]);

static void REFL_Measure(void);

static void REFL_StateMachine(void);

static void ReflTask(void* pvParameters);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static const Refl_Cfg_t *pReflCfg = NULL;
static volatile ReflStateType reflState = REFL_STATE_INIT;

static LDD_TDeviceData *timerHandle;
static xSemaphoreHandle mutexHandle;
static bool REFL_IsEnabled = TRUE;

#if REFL_START_STOP_CALIB
  static xSemaphoreHandle REFL_StartStopSem = NULL;
#endif

static NVM_ReflCalibData_t *SensorCalibMinMaxTmpPtr = NULL; /* pointer to temprory calibrated data */
static NVM_ReflCalibData_t SensorCalibMinMax={0}; /* calibration data */

static SensorTimeType SensorRaw[NUM_OF_REFL_SENSORS]; /* raw sensor values */
static SensorTimeType SensorCalibrated[NUM_OF_REFL_SENSORS]; /* 0 means white/min value, 1000 means black/max value */

static bool REFL_LedOn = TRUE;
static int16_t reflCenterLineVal=0; /* 0 means no line, >0 means line is below sensor 0, 1000 below sensor 1 and so on */
static REFL_LineKind reflLineKind = REFL_LINE_NONE;
static uint16_t reflLineWidth = 0;

static const SensorFctType SensorFctArray[NUM_OF_REFL_SENSORS] = {
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
static bool REFL_MeasureRaw(SensorTimeType raw[NUM_OF_REFL_SENSORS], RefCnt_TValueType timeoutCntVal) {
  uint8_t cnt; /* number of sensor */
  uint8_t i;
  bool isTimeout = FALSE;
  RefCnt_TValueType timerVal;

  if (!REFL_IsEnabled) {
    return ERR_DISABLED;
  }
  (void)FRTOS1_xSemaphoreTake(mutexHandle, portMAX_DELAY);

  if (REFL_LedOn) {
    IR_on(TRUE); /* IR LED's on */
    WAIT1_Waitus(200);
  }

  for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
    SensorFctArray[i].SetOutput(); /* turn I/O line as output */
    SensorFctArray[i].SetVal(); /* put high */
    raw[i] = MAX_SENSOR_VALUE;
  }
  WAIT1_Waitus(50); /* give at least 10 us to charge the capacitor */
  taskENTER_CRITICAL();
  for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
    SensorFctArray[i].SetInput(); /* turn I/O line as input */
  }
  (void)RefCnt_ResetCounter(timerHandle); /* reset timer counter */
  do {
    cnt = 0;
    timerVal = RefCnt_GetCounterValue(timerHandle);
    if (timerVal>timeoutCntVal) {
      isTimeout = TRUE;
      break; /* get out of while loop */
    }
    for(i=0;i < NUM_OF_REFL_SENSORS;i++) {
      if (raw[i]==MAX_SENSOR_VALUE) { /* not measured yet? */
        if (SensorFctArray[i].GetVal()==0) {
          raw[i] = timerVal;
        }
      } else { /* have value */
        cnt++;
      }
    }
  } while(cnt!=NUM_OF_REFL_SENSORS);
  taskEXIT_CRITICAL();

  if (REFL_LedOn) {
    IR_on(FALSE); /* IR LED's off */
  }

  (void)FRTOS1_xSemaphoreGive(mutexHandle);
  if (isTimeout) {
    for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
      if (raw[i]==MAX_SENSOR_VALUE) { /* not measured yet? */
        if (SensorCalibMinMax.maxVal[i] != 0) { //TODO
          raw[i] = SensorCalibMinMax.maxVal[i]; /* use calibrated max value */
        } else {
          raw[i] = timeoutCntVal; /* set to timeout value */
        }
      }
    } /* for */
    return ERR_OVERFLOW;
  } else {
    return ERR_OK;
  }
}

static void REFL_CalibrateMinMax(SensorTimeType min[NUM_OF_REFL_SENSORS], SensorTimeType max[NUM_OF_REFL_SENSORS], SensorTimeType raw[NUM_OF_REFL_SENSORS]) {
  int i;

  if (REFL_MeasureRaw(raw, REFL_TIMEOUT_US_TO_TICKS(pReflCfg->measTimeOutUS))==ERR_OK) { /* if timeout, do not count values */
    for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
      if (raw[i] < min[i]) {
        min[i] = raw[i];
      }
      if (raw[i]> max[i]) {
        max[i] = raw[i];
      }
    }
  }
}

static void ReadCalibrated(SensorTimeType calib[NUM_OF_REFL_SENSORS], SensorTimeType raw[NUM_OF_REFL_SENSORS]) {
  int i;
  int32_t x, denominator;
  RefCnt_TValueType max;

  max = 0; /* determine maximum timeout value */
  for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
    if (SensorCalibMinMax.maxVal[i]>max) {
      max = SensorCalibMinMax.maxVal[i];
    }
  }
  if (max > REFL_TIMEOUT_US_TO_TICKS(pReflCfg->measTimeOutUS)) { /* limit to timeout value */
    max = REFL_TIMEOUT_US_TO_TICKS(pReflCfg->measTimeOutUS);
  }
  (void)REFL_MeasureRaw(raw, max);
  if (SensorCalibMinMax.maxVal[0] == 0) { /* no calibration data? */
    return;
  }
  for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
    x = 0;
    denominator = SensorCalibMinMax.maxVal[i]-SensorCalibMinMax.minVal[i];
    if (denominator!=0) {
      x = (((int32_t)raw[i]-SensorCalibMinMax.minVal[i])*1000)/denominator;
    }
    if (x<0) {
      x = 0;
    } else if (x>1000) {
      x = 1000;
    }
    calib[i] = x;
  }
}

/*
 * Operates the same as read calibrated, but also returns an
 * estimated position of the robot with respect to a line. The
 * estimate is made using a weighted average of the sensor indices
 * multiplied by 1000, so that a return value of 1000 indicates that
 * the line is directly below sensor 0, a return value of 2000
 * indicates that the line is directly below sensor 1, 2000
 * indicates that it's below sensor 2000, etc. Intermediate
 * values indicate that the line is between two sensors. The
 * formula is:
 *
 * 1000*value0 + 2000*value1 + 3000*value2 + ...
 * --------------------------------------------
 * value0 + value1 + value2 + ...
 *
 * By default, this function assumes a dark line (high values)
 * surrounded by white (low values). If your line is light on
 * black, set the optional second argument white_line to true. In
 * this case, each sensor value will be replaced by (1000-value)
 * before the averaging.
 */

static int ReadLine(SensorTimeType calib[NUM_OF_REFL_SENSORS], SensorTimeType raw[NUM_OF_REFL_SENSORS], Refl_LineBW_t lineBW_) {
  int i;
  unsigned long avg; /* this is for the weighted total, which is long */
  /* before division */
  unsigned int sum; /* this is for the denominator which is <= 64000 */
  unsigned int mul; /* multiplication factor, 0, 1000, 2000, 3000 ... */
  int value;
#if RETURN_LAST_VALUE
  bool on_line = FALSE;
  static int last_value=0; /* assume initially that the line is left. */
#endif

  (void)raw; /* unused */
  avg = 0;
  sum = 0;
  mul = 1000;
  for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
    value = calib[i];
    if(REFL_LINE_WHITE == lineBW_) {
      value = 1000-value;
    }
    /* only average in values that are above a noise threshold */
    if(value > pReflCfg->minNoiseVal) {
      avg += ((long)value)*mul;
      sum += value;
    }
    mul += 1000;
  }
#if RETURN_LAST_VALUE
  /* keep track of whether we see the line at all */
  if(value > REFL_MIN_LINE_VAL) {
    on_line = TRUE;
  }
  if(!on_line) { /* If it last read to the left of center, return 0. */
    if(last_value < endIdx*1000/2) {
      return 0;
    } else { /* If it last read to the right of center, return the max. */
      return endIdx*1000;
    }
  }
  last_value = avg/sum;
  return last_value;
#else
  if (sum>0) {
    return avg/sum;
  } else {
    return avg;
  }
#endif
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

static REFL_LineKind ReadLineKind(SensorTimeType val[NUM_OF_REFL_SENSORS]) {
  uint32_t sum, sumLeft, sumRight, outerLeft, outerRight;
  int i;

  /* check if robot is in the air or does see something */
  if (SensorsSaturated()) {
    return REFL_LINE_AIR;
  }
  for(i = 0; i < NUM_OF_REFL_SENSORS; i++) {
    if (val[i] < pReflCfg->minLineVal) { /* smaller value? White seen! */
      break;
    }
  }
  if (i==NUM_OF_REFL_SENSORS) { /* all sensors see 'black' */
    return REFL_LINE_FULL;
  }
  /* check the line type */
  sum = 0; sumLeft = 0; sumRight = 0;
  for(i=0;i<NUM_OF_REFL_SENSORS;i++) {
    if (val[i] > pReflCfg->minLineVal) { /* count only line values */
      sum += val[i];
      if ( i < NUM_OF_REFL_SENSORS / 2 ) {

        sumLeft += val[i];

      } else {

        sumRight += val[i];

      }
    }
  }

  outerLeft = val[0];
  outerRight = val[NUM_OF_REFL_SENSORS-1];



  if ( outerLeft >= pReflCfg->minLineVal && outerRight < pReflCfg->minLineVal && sumLeft>MIN_LEFT_RIGHT_SUM && sumRight<MIN_LEFT_RIGHT_SUM)
  {
    return REFL_LINE_LEFT; /* line going to the left side */
  }
  else if (outerLeft < pReflCfg->minLineVal && outerRight >= pReflCfg->minLineVal && sumRight>MIN_LEFT_RIGHT_SUM && sumLeft<MIN_LEFT_RIGHT_SUM)
  {
    return REFL_LINE_RIGHT; /* line going to the right side */
  }
  else if (outerLeft >= pReflCfg->minLineVal && outerRight >= pReflCfg->minLineVal && sumRight>MIN_LEFT_RIGHT_SUM && sumLeft>MIN_LEFT_RIGHT_SUM)
  {
    return REFL_LINE_FULL; /* full line */
  }
  else if (sumRight==0 && sumLeft==0 && sum == 0) {
    return REFL_LINE_NONE; /* no line */
  } else {
    return REFL_LINE_STRAIGHT; /* straight line forward */
  }
}


static uint16_t CalculateReflLineWidth(SensorTimeType calib[NUM_OF_REFL_SENSORS]) {
  int32_t val;
  int i;

  val = 0;
  for(i=0;i<NUM_OF_REFL_SENSORS;i++)
  {
    if (calib[i] >= pReflCfg->minNoiseVal) /* sensor not seeing anything? */
    {
      val += calib[i];
    }
  }
  return (uint16_t)val;
}

static void REFL_Measure(void) {
  ReadCalibrated(SensorCalibrated, SensorRaw);
  reflCenterLineVal = ReadLine(SensorCalibrated, SensorRaw, pReflCfg->lineBW);
  reflLineWidth = CalculateReflLineWidth(SensorCalibrated);
  if (reflState==REFL_STATE_READY)
  {
    reflLineKind = ReadLineKind(SensorCalibrated);
  }
}


static void REFL_StateMachine(void) {
  int i;

  switch (reflState)
  {
    case REFL_STATE_INIT:
      if (NVM_Read_ReflCalibData(&SensorCalibMinMax) == ERR_OK)  /* use calibration data from FLASH */
      {
        reflState = REFL_STATE_READY;
      } else
      {
    	SH_SENDSTR((unsigned char*)"no calibration data present.\r\n");
        reflState = REFL_STATE_NOT_CALIBRATED;
      }
      break;

    case REFL_STATE_NOT_CALIBRATED:
      FRTOS1_vTaskDelay(80/portTICK_PERIOD_MS); /* no need to sample that fast: this gives 80+20=100 ms */
      (void)REFL_MeasureRaw(SensorRaw, REFL_TIMEOUT_US_TO_TICKS(pReflCfg->measTimeOutUS));
#if REFL_START_STOP_CALIB
      if (FRTOS1_xSemaphoreTake(REFL_StartStopSem, 0)==pdTRUE)
      {
        reflState = REFL_STATE_START_CALIBRATION;
      }
#endif
      break;

    case REFL_STATE_START_CALIBRATION:
      SH_SENDSTR((unsigned char*)"start calibration...\r\n");

      if (SensorCalibMinMaxTmpPtr != NULL)
      {
        reflState = REFL_STATE_INIT; /* error case */
        break;
      }
      SensorCalibMinMaxTmpPtr = FRTOS1_pvPortMalloc(sizeof(NVM_ReflCalibData_t));
      if (SensorCalibMinMaxTmpPtr != NULL)  /* success */
      {
        for(i=0;i<NUM_OF_REFL_SENSORS;i++)
        {
          SensorCalibMinMaxTmpPtr->minVal[i] = MAX_SENSOR_VALUE;
          SensorCalibMinMaxTmpPtr->maxVal[i] = 0;
          SensorCalibrated[i] = 0;
        }
        reflState = REFL_STATE_CALIBRATING;
      } else
      {
        reflState = REFL_STATE_INIT; /* error case */
      }
      break;

    case REFL_STATE_CALIBRATING:
      FRTOS1_vTaskDelay(80/portTICK_PERIOD_MS); /* no need to sample that fast: this gives 80+20=100 ms */
      if (SensorCalibMinMaxTmpPtr != NULL) /* safety check */
      {
        REFL_CalibrateMinMax(SensorCalibMinMaxTmpPtr->minVal, SensorCalibMinMaxTmpPtr->maxVal, SensorRaw);
      } else
      {
        reflState = REFL_STATE_INIT; /* error case */
        break;
      }

      (void)BUZ_Beep(100, 50);


#if REFL_START_STOP_CALIB
      if (FRTOS1_xSemaphoreTake(REFL_StartStopSem, 0)==pdTRUE)
      {
        reflState = REFL_STATE_STOP_CALIBRATION;
      }
#endif
      break;

    case REFL_STATE_STOP_CALIBRATION:

      SH_SENDSTR((unsigned char*)"...stopping calibration.\r\n");

      reflState = REFL_STATE_SAVE_CALIBRATION;
      break;

    case REFL_STATE_SAVE_CALIBRATION:
      if(NVM_Save_ReflCalibData(SensorCalibMinMaxTmpPtr) != ERR_OK)
      {
    	  SH_SENDSTR((unsigned char*)"failed to save calib data");
      }
      /* free memory */
      FRTOS1_vPortFree(SensorCalibMinMaxTmpPtr);
      SensorCalibMinMaxTmpPtr = NULL;
      if(NVM_Read_ReflCalibData(&SensorCalibMinMax) == ERR_OK)
      {
    	  SH_SENDSTR((unsigned char*)"calibration data saved");

    	  reflState = REFL_STATE_READY;
      }else
      {
    	  reflState = REFL_STATE_INIT;
      }
      break;


    case REFL_STATE_READY:
      REFL_Measure();

#if REFL_START_STOP_CALIB
      if (FRTOS1_xSemaphoreTake(REFL_StartStopSem, 0)==pdTRUE)
      {
        reflState = REFL_STATE_START_CALIBRATION;
      }
#endif
      break;
  } /* switch */
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
ReflStateType REFL_GetReflState(void){
	return reflState;
}

SensorTimeType REFL_GetRawSensorValue(const uint8 i){
	return SensorRaw[i];
}

SensorTimeType REFL_GetCalibratedSensorValue(const uint8 i){
	return SensorCalibrated[i];
}
int16_t REFL_GetReflLineValue(void){
	return reflCenterLineVal;
}

REFL_LineKind REFL_GetReflLineKind(void){
	return reflLineKind;
}

int16_t REFL_GetReflLineWidth(void){
	return reflLineWidth;
}
NVM_ReflCalibData_t* REFL_GetCalibMinMaxPtr(void){
	return &SensorCalibMinMax;
}

bool REFL_IsReflEnabled(void){
	return REFL_IsEnabled;
}

void REFL_SetReflEnabled(bool isEnabled){
	if(REFL_IsEnabled != isEnabled)
	{
	REFL_IsEnabled = isEnabled;
	}
}

bool REFL_CanUseSensor(void) {
  return reflState==REFL_STATE_READY;
}

uint16_t REFL_GetLineValue(bool *onLine) {
  *onLine = reflCenterLineVal>0 && reflCenterLineVal<REFL_MAX_LINE_VALUE;
  return reflCenterLineVal;
}

bool REFL_IsRefEnabled(void){
	return REFL_IsEnabled;
}

bool REFL_IsLedOn(void){
	return REFL_LedOn;
}

void REFL_SetLedOn(bool isOn){
	if(REFL_LedOn != isOn)
	{
	REFL_LedOn = isOn;
	}
}


#if REFL_START_STOP_CALIB
void REFL_CalibrateStartStop(void) {
  REFL_IsEnabled = TRUE;
  if (reflState==REFL_STATE_NOT_CALIBRATED || reflState==REFL_STATE_CALIBRATING || reflState==REFL_STATE_READY)
  {
    (void)xSemaphoreGive(REFL_StartStopSem);
  }
}
#endif

void REFL_GetSensorValues(uint16_t *values, int nofValues) {
  uint8_t i = 0u;

  for(i = 0u; i < nofValues && i < NUM_OF_REFL_SENSORS; i++)
  {
    values[i] = SensorCalibrated[i];
  }
}

StdRtn_t REFL_Read_ReflCfg(Refl_Cfg_t *pCfg_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if( NULL != pCfg_)
	{
		if( NULL != pReflCfg)
		{
			*pCfg_ = *pReflCfg;
			retVal = ERR_OK;
		}
		else
		{
			retVal = ERR_PARAM_DATA;
		}
	}
	return retVal;
}

uint8_t REFL_Get_NumOfSensors(void)
{
	return NUM_OF_REFL_SENSORS;
}

void REFL_Init(void) {
	pReflCfg = Get_pReflCfg();

	// TODO ab hier
	if(pReflCfg == NULL)
	{
	    for(;;){} /* error */
	}

#if REFL_START_STOP_CALIB
  FRTOS1_vSemaphoreCreateBinary(REFL_StartStopSem);
  if (REFL_StartStopSem==NULL) /* semaphore creation failed */
  {
    for(;;){} /* error */
  }
  (void)FRTOS1_xSemaphoreTake(REFL_StartStopSem, 0); /* empty token */
  FRTOS1_vQueueAddToRegistry(REFL_StartStopSem, "ReflStartStopSem");
#endif

  reflState = REFL_STATE_INIT;

  reflLineKind = REFL_LINE_NONE;

  reflCenterLineVal = 0;
  IR_on(TRUE);
  REFL_LedOn = TRUE; /* IR LED's on */
  //TODO
  REFL_IsEnabled = TRUE;

  mutexHandle = FRTOS1_xSemaphoreCreateMutex();
  if (mutexHandle==NULL)
  {
    for(;;);
  }
  FRTOS1_vQueueAddToRegistry(mutexHandle, "ReflSem");
  timerHandle = RefCnt_Init(NULL);
}

void REFL_MainFct(void) {
    REFL_StateMachine();
}



#ifdef MASTER_refl_C_
#undef MASTER_refl_C_
#endif /* !MASTER_refl_C_ */
