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
/**
 * REF_SENSOR_TIMEOUT_US translated into timeout ticks
 */
#define REFL_TIMEOUT_US_TO_TICKS(timOutUS_)      ((RefCnt_CNT_INP_FREQ_U_0/1000)*timOutUS_)/1000 /* REF_SENSOR_TIMEOUT_US translated into timeout ticks */




/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @typedef
 *
 * @enum
 */
typedef enum Refl_LineBW_e {
	 REFL_LINE_WHITE = 0x00 /**< REFL_LINE_WHITE */
	,REFL_LINE_BLACK        /**< REFL_LINE_BLACK */
	,REFL_LINE_BW_CNT
} Refl_LineBW_t;

/**
 * @typedef
 *
 * @struct
 */
typedef struct Refl_Cfg_s {
	 uint16_t minNoiseVal;
	 uint16_t minLineVal;
	 Refl_LineBW_t lineBW;
	 uint16_t measTimeOutUS;
 } Refl_Cfg_t;

 /**
  *
  */
typedef enum REFL_LineKind_e {
  REFL_LINE_NONE=0,     /**< no line, sensors do not see a line */
  REFL_LINE_STRAIGHT=1, /**< forward line |, sensors see a line underneath */
  REFL_LINE_LEFT=2,     /**< left half of sensors see line */
  REFL_LINE_RIGHT=3,    /**< right half of sensors see line */
  REFL_LINE_FULL=4,     /**< all sensors see a line */
  REFL_LINE_AIR=5,      /**< all sensors have a timeout value. Robot is not on ground at all? */
  REFL_NOF_LINES        /**< Sentinel */
} REFL_LineKind;

/**
 *
 */
typedef enum {
  REFL_STATE_INIT,             //!< REFL_STATE_INIT
  REFL_STATE_NOT_CALIBRATED,   //!< REFL_STATE_NOT_CALIBRATED
  REFL_STATE_START_CALIBRATION,//!< REFL_STATE_START_CALIBRATION
  REFL_STATE_CALIBRATING,      //!< REFL_STATE_CALIBRATING
  REFL_STATE_STOP_CALIBRATION, //!< REFL_STATE_STOP_CALIBRATION
  REFL_STATE_SAVE_CALIBRATION, //!< REFL_STATE_SAVE_CALIBRATION
  REFL_STATE_READY             //!< REFL_STATE_READY
} ReflStateType;

/**
 *
 */
typedef struct SensorFctType_ {
  void (*SetOutput)(void);
  void (*SetInput)(void);
  void (*SetVal)(void);
  bool (*GetVal)(void);
} SensorFctType;

/**
 *
 */
typedef uint16_t SensorTimeType;



/*============================= >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ StdRtn_t REFL_Read_ReflCfg(Refl_Cfg_t *pCfg_);

EXTERNAL_ uint8_t REFL_Get_NumOfSensors(void);

EXTERNAL_ ReflStateType REFL_GetReflState(void);

EXTERNAL_ bool REFL_IsReflEnabled(void);

EXTERNAL_ void REFL_SetReflEnabled(bool isEnabled);

EXTERNAL_ NVM_ReflCalibData_t* REFL_GetCalibMinMaxPtr(void);

EXTERNAL_ void REFL_SetLedOn(bool isOn);

EXTERNAL_ int16_t REFL_GetReflLineWidth(void);

EXTERNAL_ int16_t REFL_GetReflLineValue(void);

EXTERNAL_ SensorTimeType REFL_GetCalibratedSensorValue(const uint8 i);

EXTERNAL_ SensorTimeType REFL_GetRawSensorValue(const uint8 i);

EXTERNAL_ void REFL_CalibrateStartStop(void);

EXTERNAL_ void REFL_GetSensorValues(uint16_t *values, int nofValues);

EXTERNAL_ REFL_LineKind REFL_GetReflLineKind(void);

EXTERNAL_ int16_t REFL_GetReflLineWidth(void);

EXTERNAL_ uint16_t REFL_GetLineValue(bool *onLine);

EXTERNAL_ bool REFL_IsLedOn(void);

/*!
 * \brief Function to find out if we can use the sensor (means: it is calibrated and not currently calibrating)
 * \return TRUE if the sensor is ready.
 */
EXTERNAL_ bool REFL_CanUseSensor(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !REFL_API_H_ */

