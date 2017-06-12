/***********************************************************************************************//**
 * @file		refl_api.h
 * @ingroup		refl
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	Simon Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef REFL_API_H_
#define REFL_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "PE_Types.h"
#include "RefCnt.h"
#include "nvm_api.h"

#ifdef MASTER_refl_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup refl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
#define REF_MIN_LINE_VAL      0x120   /* minimum value indicating a line */
//#define REF_MIN_NOISE_VAL      0x40   /* values below this are not added to the weighted sum */
#define REF_MIN_NOISE_VAL      0x80   /* values below this are not added to the weighted sum */
#define REF_SENSOR_TIMEOUT_US  3500   /* after this time, consider no reflection (black). Must be smaller than the timeout period of the RefCnt timer! */
#define REF_NOF_SENSORS 6
#define REF_MIDDLE_LINE_VALUE  ((REF_NOF_SENSORS+1)*1000/2)
#define REF_MAX_LINE_VALUE     ((REF_NOF_SENSORS+1)*1000) /* maximum value for REF_GetLine() */
#define REF_TIMEOUT_TICKS       	((RefCnt_CNT_INP_FREQ_U_0/1000)*REF_SENSOR_TIMEOUT_US)/1000 /* REF_SENSOR_TIMEOUT_US translated into timeout ticks */


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum REF_LineKind_e {
  REF_LINE_NONE=0,     /* no line, sensors do not see a line */
  REF_LINE_STRAIGHT=1, /* forward line |, sensors see a line underneath */
  REF_LINE_LEFT=2,     /* left half of sensors see line */
  REF_LINE_RIGHT=3,    /* right half of sensors see line */
  REF_LINE_FULL=4,     /* all sensors see a line */
  REF_LINE_AIR=5,      /* all sensors have a timeout value. Robot is not on ground at all? */
  REF_NOF_LINES        /* Sentinel */
} REF_LineKind;

typedef enum {
  REF_STATE_INIT,
  REF_STATE_NOT_CALIBRATED,
  REF_STATE_START_CALIBRATION,
  REF_STATE_CALIBRATING,
  REF_STATE_STOP_CALIBRATION,
  REF_STATE_SAVE_CALIBRATION,
  REF_STATE_READY
} RefStateType;

typedef struct SensorFctType_ {
  void (*SetOutput)(void);
  void (*SetInput)(void);
  void (*SetVal)(void);
  bool (*GetVal)(void);
} SensorFctType;

typedef uint16_t SensorTimeType;


/*============================= >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ void REF_GetSensorValues(uint16_t *values, int nofValues);

EXTERNAL_ REF_LineKind REF_GetRefLineKind(void);

EXTERNAL_ int16_t REF_GetRefLineWidth(void);

EXTERNAL_ uint16_t REF_GetLineValue(bool *onLine);

EXTERNAL_ bool REF_IsRefEnabled(void);

EXTERNAL_ bool REF_IsLedOn(void);

/*!
 * \brief Function to find out if we can use the sensor (means: it is calibrated and not currently calibrating)
 * \return TRUE if the sensor is ready.
 */
EXTERNAL_ bool REF_CanUseSensor(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !REFL_API_H_ */

