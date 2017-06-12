/***********************************************************************************************//**
 * @file		refl.c
 * @ingroup		refl
 * @brief 		Implements driver software for reflectance sensor array
 *
 * <This is a detailed description.>
 *
 * @author 	Simon Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define MASTER_refl_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "refl.h"
#include "refl_api.h"
#include "refl_clshdlr.h"
#include "sh_api.h"
#include "buz_api.h"
#include "nvm_api.h"


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
#define REF_USE_WHITE_LINE  		0  /* if set to 1, then the robot is using a white (on black) line, otherwise a black (on white) line */
#define REF_START_STOP_CALIB      	1 /* start/stop calibration commands */
#define MAX_SENSOR_VALUE  			((SensorTimeType)-1) //0xFFFF

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

static bool REF_MeasureRaw(SensorTimeType raw[REF_NOF_SENSORS], RefCnt_TValueType timeoutCntVal);

static void REF_CalibrateMinMax(SensorTimeType min[REF_NOF_SENSORS], SensorTimeType max[REF_NOF_SENSORS], SensorTimeType raw[REF_NOF_SENSORS]);

static void ReadCalibrated(SensorTimeType calib[REF_NOF_SENSORS], SensorTimeType raw[REF_NOF_SENSORS]);

static int ReadLine(SensorTimeType calib[REF_NOF_SENSORS], SensorTimeType raw[REF_NOF_SENSORS], bool white_line);

static bool SensorsSaturated(void);

static REF_LineKind ReadLineKind(SensorTimeType val[REF_NOF_SENSORS]);

static uint16_t CalculateRefLineWidth(SensorTimeType calib[REF_NOF_SENSORS]);

static void REF_Measure(void);

static void REF_StateMachine(void);

static void ReflTask(void* pvParameters);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static volatile RefStateType refState = REF_STATE_INIT;

static LDD_TDeviceData *timerHandle;
static xSemaphoreHandle mutexHandle;
static bool REF_IsEnabled = TRUE;

#if REF_START_STOP_CALIB
  static xSemaphoreHandle REF_StartStopSem = NULL;
#endif

static NVM_ReflCalibData_t *SensorCalibMinMaxTmpPtr = NULL; /* pointer to temprory calibrated data */
static NVM_ReflCalibData_t SensorCalibMinMax={0}; /* calibration data */

static SensorTimeType SensorRaw[REF_NOF_SENSORS]; /* raw sensor values */
static SensorTimeType SensorCalibrated[REF_NOF_SENSORS]; /* 0 means white/min value, 1000 means black/max value */

static bool REF_LedOn = TRUE;
static int16_t refCenterLineVal=0; /* 0 means no line, >0 means line is below sensor 0, 1000 below sensor 1 and so on */
static REF_LineKind refLineKind = REF_LINE_NONE;
static uint16_t refLineWidth = 0;

static const SensorFctType SensorFctArray[REF_NOF_SENSORS] = {
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
static bool REF_MeasureRaw(SensorTimeType raw[REF_NOF_SENSORS], RefCnt_TValueType timeoutCntVal) {
  uint8_t cnt; /* number of sensor */
  uint8_t i;
  bool isTimeout = FALSE;
  RefCnt_TValueType timerVal;

  if (!REF_IsEnabled) {
    return ERR_DISABLED;
  }
  (void)FRTOS1_xSemaphoreTake(mutexHandle, portMAX_DELAY);
#if 1 /* always on */
  if (REF_LedOn) {
    IR_on(TRUE); /* IR LED's on */
    WAIT1_Waitus(200);
  }
#endif
  for(i=0;i<REF_NOF_SENSORS;i++) {
    SensorFctArray[i].SetOutput(); /* turn I/O line as output */
    SensorFctArray[i].SetVal(); /* put high */
    raw[i] = MAX_SENSOR_VALUE;
  }
  WAIT1_Waitus(50); /* give at least 10 us to charge the capacitor */
  taskENTER_CRITICAL();
  for(i=0;i<REF_NOF_SENSORS;i++) {
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
    for(i=0;i<REF_NOF_SENSORS;i++) {
      if (raw[i]==MAX_SENSOR_VALUE) { /* not measured yet? */
        if (SensorFctArray[i].GetVal()==0) {
          raw[i] = timerVal;
        }
      } else { /* have value */
        cnt++;
      }
    }
  } while(cnt!=REF_NOF_SENSORS);
  taskEXIT_CRITICAL();
#if 1 /* always on */
  if (REF_LedOn) {
    IR_on(FALSE); /* IR LED's off */
  }
#endif
  (void)FRTOS1_xSemaphoreGive(mutexHandle);
  if (isTimeout) {
    for(i=0;i<REF_NOF_SENSORS;i++) {
      if (raw[i]==MAX_SENSOR_VALUE) { /* not measured yet? */
        if (SensorCalibMinMax.maxVal[0] != 0) { //TODO
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

static void REF_CalibrateMinMax(SensorTimeType min[REF_NOF_SENSORS], SensorTimeType max[REF_NOF_SENSORS], SensorTimeType raw[REF_NOF_SENSORS]) {
  int i;

  if (REF_MeasureRaw(raw, REF_TIMEOUT_TICKS)==ERR_OK) { /* if timeout, do not count values */
    for(i=0;i<REF_NOF_SENSORS;i++) {
      if (raw[i] < min[i]) {
        min[i] = raw[i];
      }
      if (raw[i]> max[i]) {
        max[i] = raw[i];
      }
    }
  }
}

static void ReadCalibrated(SensorTimeType calib[REF_NOF_SENSORS], SensorTimeType raw[REF_NOF_SENSORS]) {
  int i;
  int32_t x, denominator;
  RefCnt_TValueType max;

  max = 0; /* determine maximum timeout value */
  for(i=0;i<REF_NOF_SENSORS;i++) {
    if (SensorCalibMinMax.maxVal[i]>max) {
      max = SensorCalibMinMax.maxVal[i];
    }
  }
  if (max > REF_TIMEOUT_TICKS) { /* limit to timeout value */
    max = REF_TIMEOUT_TICKS;
  }
  (void)REF_MeasureRaw(raw, max);
  if (SensorCalibMinMax.maxVal[0] == 0) { /* no calibration data? */
    return;
  }
  for(i=0;i<REF_NOF_SENSORS;i++) {
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
#define RETURN_LAST_VALUE  0
static int ReadLine(SensorTimeType calib[REF_NOF_SENSORS], SensorTimeType raw[REF_NOF_SENSORS], bool white_line) {
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
  for(i=0;i<REF_NOF_SENSORS;i++) {
    value = calib[i];
    if(white_line) {
      value = 1000-value;
    }
    /* only average in values that are above a noise threshold */
    if(value > REF_MIN_NOISE_VAL) {
      avg += ((long)value)*mul;
      sum += value;
    }
    mul += 1000;
  }
#if RETURN_LAST_VALUE
  /* keep track of whether we see the line at all */
  if(value > REF_MIN_LINE_VAL) {
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
  for(i=0;i<REF_NOF_SENSORS;i++) {
    if (SensorRaw[i]==MAX_SENSOR_VALUE) { /* sensor not seeing anything? */
      cnt++;
    }
  }
  return (cnt==REF_NOF_SENSORS); /* all sensors see raw max value: not on the ground? */
#else
  return FALSE;
#endif
}

static REF_LineKind ReadLineKind(SensorTimeType val[REF_NOF_SENSORS]) {
  uint32_t sum, sumLeft, sumRight, outerLeft, outerRight;
  int i;

  /* check if robot is in the air or does see something */
  if (SensorsSaturated()) {
    return REF_LINE_AIR;
  }
  for(i = 0; i < REF_NOF_SENSORS; i++) {
    if (val[i]<REF_MIN_LINE_VAL) { /* smaller value? White seen! */
      break;
    }
  }
  if (i==REF_NOF_SENSORS) { /* all sensors see 'black' */
    return REF_LINE_FULL;
  }
  /* check the line type */
  sum = 0; sumLeft = 0; sumRight = 0;
  for(i=0;i<REF_NOF_SENSORS;i++) {
    if (val[i]>REF_MIN_LINE_VAL) { /* count only line values */
      sum += val[i];
      if (i<REF_NOF_SENSORS/2) {

        sumLeft += val[i];

      } else {

        sumRight += val[i];

      }
    }
  }

  outerLeft = val[0];
  outerRight = val[REF_NOF_SENSORS-1];

  #define MIN_LEFT_RIGHT_SUM   ((REF_NOF_SENSORS*1000)/4) /* 1/4 of full sensor values */

  if (outerLeft>=REF_MIN_LINE_VAL && outerRight<REF_MIN_LINE_VAL && sumLeft>MIN_LEFT_RIGHT_SUM && sumRight<MIN_LEFT_RIGHT_SUM) {
    return REF_LINE_LEFT; /* line going to the left side */
  } else if (outerLeft<REF_MIN_LINE_VAL && outerRight>=REF_MIN_LINE_VAL && sumRight>MIN_LEFT_RIGHT_SUM && sumLeft<MIN_LEFT_RIGHT_SUM) {
    return REF_LINE_RIGHT; /* line going to the right side */
  } else if (outerLeft>=REF_MIN_LINE_VAL && outerRight>=REF_MIN_LINE_VAL && sumRight>MIN_LEFT_RIGHT_SUM && sumLeft>MIN_LEFT_RIGHT_SUM) {
    return REF_LINE_FULL; /* full line */
  } else if (sumRight==0 && sumLeft==0 && sum == 0) {
    return REF_LINE_NONE; /* no line */
  } else {
    return REF_LINE_STRAIGHT; /* straight line forward */
  }
}


static uint16_t CalculateRefLineWidth(SensorTimeType calib[REF_NOF_SENSORS]) {
  int32_t val;
  int i;

  val = 0;
  for(i=0;i<REF_NOF_SENSORS;i++)
  {
    if (calib[i]>=REF_MIN_NOISE_VAL) /* sensor not seeing anything? */
    {
      val += calib[i];
    }
  }
  return (uint16_t)val;
}

static void REF_Measure(void) {
  ReadCalibrated(SensorCalibrated, SensorRaw);
  refCenterLineVal = ReadLine(SensorCalibrated, SensorRaw, REF_USE_WHITE_LINE);
  refLineWidth = CalculateRefLineWidth(SensorCalibrated);
  if (refState==REF_STATE_READY)
  {
    refLineKind = ReadLineKind(SensorCalibrated);
  }
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
RefStateType REF_GetRefState(void){
	return refState;
}

SensorTimeType REF_GetRawSensorValue(const uint8 i){
	return SensorRaw[i];
}

SensorTimeType REF_GetCalibratedSensorValue(const uint8 i){
	return SensorCalibrated[i];
}
int16_t REF_GetRefLineValue(void){
	return refCenterLineVal;
}

REF_LineKind REF_GetRefLineKind(void){
	return refLineKind;
}

int16_t REF_GetRefLineWidth(void){
	return refLineWidth;
}
NVM_ReflCalibData_t* REF_GetCalibMinMaxPtr(void){
	return &SensorCalibMinMax;
}

void REF_SetRefEnabled(bool isEnabled){
	if(REF_IsEnabled != isEnabled)
	{
	REF_IsEnabled = isEnabled;
	}
}

bool REF_CanUseSensor(void) {
  return refState==REF_STATE_READY;
}

uint16_t REF_GetLineValue(bool *onLine) {
  *onLine = refCenterLineVal>0 && refCenterLineVal<REF_MAX_LINE_VALUE;
  return refCenterLineVal;
}

bool REF_IsRefEnabled(void){
	return REF_IsEnabled;
}

bool REF_IsLedOn(void){
	return REF_LedOn;
}

void REF_SetLedOn(bool isOn){
	if(REF_LedOn != isOn)
	{
	REF_LedOn = isOn;
	}
}


#if REF_START_STOP_CALIB
void REF_CalibrateStartStop(void) {
  REF_IsEnabled = TRUE;
  if (refState==REF_STATE_NOT_CALIBRATED || refState==REF_STATE_CALIBRATING || refState==REF_STATE_READY)
  {
    (void)xSemaphoreGive(REF_StartStopSem);
  }
}
#endif

void REF_GetSensorValues(uint16_t *values, int nofValues) {
  int i;

  for(i=0;i<nofValues && i<REF_NOF_SENSORS;i++)
  {
    values[i] = SensorCalibrated[i];
  }
}

static void REF_StateMachine(void) {
  int i;

  switch (refState)
  {
    case REF_STATE_INIT:
//      SensorCalibMinMaxPtr = NVMC_GetReflectanceData();
      if (NVM_Read_ReflCalibData(&SensorCalibMinMax) == ERR_OK)
      { /* use calibration data from FLASH */
        refState = REF_STATE_READY;
      } else
      {
    	SH_SENDSTR((unsigned char*)"no calibration data present.\r\n");
        refState = REF_STATE_NOT_CALIBRATED;
      }
      break;

    case REF_STATE_NOT_CALIBRATED:
      FRTOS1_vTaskDelay(80/portTICK_PERIOD_MS); /* no need to sample that fast: this gives 80+20=100 ms */
      (void)REF_MeasureRaw(SensorRaw, REF_TIMEOUT_TICKS);
#if REF_START_STOP_CALIB
      if (FRTOS1_xSemaphoreTake(REF_StartStopSem, 0)==pdTRUE)
      {
        refState = REF_STATE_START_CALIBRATION;
      }
#endif
      break;

    case REF_STATE_START_CALIBRATION:
      SH_SENDSTR((unsigned char*)"start calibration...\r\n");

      if (SensorCalibMinMaxTmpPtr != NULL)
      {
        refState = REF_STATE_INIT; /* error case */
        break;
      }
      SensorCalibMinMaxTmpPtr = FRTOS1_pvPortMalloc(sizeof(NVM_ReflCalibData_t));
      if (SensorCalibMinMaxTmpPtr != NULL)  /* success */
      {
        for(i=0;i<REF_NOF_SENSORS;i++)
        {
          SensorCalibMinMaxTmpPtr->minVal[i] = MAX_SENSOR_VALUE;
          SensorCalibMinMaxTmpPtr->maxVal[i] = 0;
          SensorCalibrated[i] = 0;
        }
        refState = REF_STATE_CALIBRATING;
      } else
      {
        refState = REF_STATE_INIT; /* error case */
      }
      break;

    case REF_STATE_CALIBRATING:
      FRTOS1_vTaskDelay(80/portTICK_PERIOD_MS); /* no need to sample that fast: this gives 80+20=100 ms */
      if (SensorCalibMinMaxTmpPtr != NULL) /* safety check */
      {
        REF_CalibrateMinMax(SensorCalibMinMaxTmpPtr->minVal, SensorCalibMinMaxTmpPtr->maxVal, SensorRaw);
      } else
      {
        refState = REF_STATE_INIT; /* error case */
        break;
      }

      (void)BUZ_Beep(100, 50);


#if REF_START_STOP_CALIB
      if (FRTOS1_xSemaphoreTake(REF_StartStopSem, 0)==pdTRUE)
      {
        refState = REF_STATE_STOP_CALIBRATION;
      }
#endif
      break;

    case REF_STATE_STOP_CALIBRATION:

      SH_SENDSTR((unsigned char*)"...stopping calibration.\r\n");

      refState = REF_STATE_SAVE_CALIBRATION;
      break;

    case REF_STATE_SAVE_CALIBRATION:
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

    	  refState = REF_STATE_READY;
      }else
      {
    	  refState = REF_STATE_INIT;
      }
      break;


    case REF_STATE_READY:
      REF_Measure();

#if REF_START_STOP_CALIB
      if (FRTOS1_xSemaphoreTake(REF_StartStopSem, 0)==pdTRUE)
      {
        refState = REF_STATE_START_CALIBRATION;
      }
#endif
      break;
  } /* switch */
}


static void ReflTask(void *pvParameters) {
  (void)pvParameters; /* not used */
  for(;;)
  {
    REF_StateMachine();
    FRTOS1_vTaskDelay(20/portTICK_PERIOD_MS);
  }
}


void REF_Init(void) {
#if REF_START_STOP_CALIB
  FRTOS1_vSemaphoreCreateBinary(REF_StartStopSem);
  if (REF_StartStopSem==NULL) /* semaphore creation failed */
  {
    for(;;){} /* error */
  }
  (void)FRTOS1_xSemaphoreTake(REF_StartStopSem, 0); /* empty token */
  FRTOS1_vQueueAddToRegistry(REF_StartStopSem, "RefStartStopSem");
#endif

  refState = REF_STATE_INIT;

  refLineKind = REF_LINE_NONE;

  refCenterLineVal = 0;
  IR_on(TRUE); REF_LedOn = TRUE; /* IR LED's on */
  //TODO
  REF_IsEnabled = TRUE;

  mutexHandle = FRTOS1_xSemaphoreCreateMutex();
  if (mutexHandle==NULL)
  {
    for(;;);
  }
  FRTOS1_vQueueAddToRegistry(mutexHandle, "RefSem");
  timerHandle = RefCnt_Init(NULL);
  if (FRTOS1_xTaskCreate(ReflTask, "Refl", configMINIMAL_STACK_SIZE+50, NULL, tskIDLE_PRIORITY+4, NULL) != pdPASS)
  {
    for(;;){} /* error */
  }
}


#ifdef MASTER_refl_C_
#undef MASTER_refl_C_
#endif /* !MASTER_refl_C_ */
