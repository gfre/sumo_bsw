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
 * @copyright @LGPL2_1
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
#define REFL_TIMEOUT_US_TO_TICKS(timOutUS_)		( (RefCnt_CNT_INP_FREQ_U_0 / 1000u) * timOutUS_ ) / 1000u



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @typedef REFL_SnsrTime_t
 * @brief Data type definition of the sensor timing
 */
typedef uint16_t REFL_SnsrTime_t;


/**
 * @typedef REFL_LineBW_t
 * @brief Data type definition of the enumeration REFL_LineBW_e
 *
 * @enum REFL_LineBW_e
 * @brief
 */
typedef enum REFL_LineBW_e {
	 REFL_LINE_WHITE = 0x00 /**< REFL_LINE_WHITE */
	,REFL_LINE_BLACK        /**< REFL_LINE_BLACK */
	,REFL_LINE_BW_CNT
} REFL_LineBW_t;

/**
 * @typedef REFL_Cfg_t
 * @brief Data type definition of the structure REFL_Cfg_s
 *
 * @struct REFL_Cfg_s
 * @brief
 */
typedef struct REFL_Cfg_s {
	REFL_SnsrTime_t minNoiseVal;
	REFL_SnsrTime_t minLineVal;
	REFL_LineBW_t lineBW;
	REFL_SnsrTime_t measTimeOutUS;
 } REFL_Cfg_t;

 /**
  * @typedef REFL_LineKind_t
  * @brief Data type definition of the enumeration REFL_LineKind_e
  *
  * @enum
  * @brief
  */
 typedef enum REFL_LineKind_e {
   REFL_LINE_STRAIGHT = 0, 	/**< forward line |, sensors see a line underneath */
   REFL_LINE_LEFT,     		/**< left half of sensors see line */
   REFL_LINE_RIGHT,    		/**< right half of sensors see line */
   REFL_LINE_FULL,     		/**< all sensors see a line */
   REFL_LINE_AIR,      		/**< all sensors have a timeout value. Robot is not on ground at all? */
   REFL_LINE_CNT,      		/**< number of lines */
   REFL_LINE_NONE,     		/**< no line, sensors do not see a line */
 } REFL_LineKind_t;

 /**
  * @typedef REFL_Line_t
  * @brief
  *
  * @struct REFL_Line_s
  * @brief
  */
 typedef struct REFL_Line_s {
 	uint16_t center;
 	REFL_LineKind_t kind;
 	uint16_t width;
 } REFL_Line_t;


/**
 * @typedef REFL_State_t
 * @brief Data type definition of the enumeration REFL_State_e
 *
 * @enum REFL_State_e
 * @brief
 */
typedef enum REFL_State_e {
  REFL_STATE_INIT,             //!< REFL_STATE_INIT
  REFL_STATE_NOT_CALIBRATED,   //!< REFL_STATE_NOT_CALIBRATED
  REFL_STATE_START_CALIBRATION,//!< REFL_STATE_START_CALIBRATION
  REFL_STATE_CALIBRATING,      //!< REFL_STATE_CALIBRATING
  REFL_STATE_STOP_CALIBRATION, //!< REFL_STATE_STOP_CALIBRATION
  REFL_STATE_SAVE_CALIBRATION, //!< REFL_STATE_SAVE_CALIBRATION
  REFL_STATE_READY             //!< REFL_STATE_READY
} REFL_State_t;

/**
 * @typedef SnsrIOFcts_t
 * @brief Data type definition of the structure SnsrIOFcts_s
 *
 * @struct SnsrIOFcts_s
 * @brief
 */
typedef struct SnsrIOFcts_s {
  void (*SetOutput)(void);
  void (*SetInput)(void);
  void (*SetVal)(void);
  bool (*GetVal)(void);
} SnsrIOFcts_t;



/*============================= >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief
 * @param pCfg_
 * @return
 */
EXTERNAL_ StdRtn_t REFL_Read_ReflCfg(REFL_Cfg_t *pCfg_);

/**
 * @brief
 * @param dctdLine_
 * @return
 */
EXTERNAL_ StdRtn_t REFL_Read_DctdLine(REFL_Line_t *dctdLine_);

/**
 *
 * @param onLine_
 * @return
 */
EXTERNAL_ uint16_t REFL_Get_DctdLineCenter(bool *onLine_);

/**
 * @brief
 * @return
 */
EXTERNAL_ REFL_LineKind_t REFL_Get_DctdLineKind(void);

/**
 *
 * @return
 */
EXTERNAL_ uint16_t REFL_Get_DctdLineWidth(void);


/**
 * @brief
 * @return
 */
EXTERNAL_ uint8_t REFL_Get_NumOfSnsrs(void);

/**
 * @brief
 * @return
 */
EXTERNAL_ REFL_State_t REFL_Get_State(void);

/**
 * @brief
 * @return
 */
EXTERNAL_ NVM_ReflCalibData_t* REFL_Get_pCalibData(void);

/**
 * @brief
 * @return
 */
EXTERNAL_ bool REFL_Get_SwcEnbldSt(void);

/**
 *
 * @param flag_
 */
EXTERNAL_ void REFL_Set_SwcEnbldSt(bool state_);

/**
 * @brief
 * @return
 */
EXTERNAL_ bool REFL_Get_IrLedSt(void);

/**
 * @brief
 * @param state_
 */
EXTERNAL_ void REFL_Set_IrLedSt(bool state_);

/**
 * @brief
 * @param idx_
 * @return
 */
EXTERNAL_ REFL_SnsrTime_t REFL_Get_RawSnsrVal(uint8_t idx_);

/**
 * @brief
 * @param idx_
 * @return
 */
EXTERNAL_ REFL_SnsrTime_t REFL_Get_NormSnsrVal(uint8_t idx_);

/**
 * @brief Function to find out if we can use the sensor (means: it is calibrated and not currently calibrating)
 * @return TRUE if the sensor is ready.
 */
EXTERNAL_ bool REFL_CanUseSensor(void);

/**
 * @brief
 */
EXTERNAL_ void REFL_Give_Smphr4CalibStartStop(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !REFL_API_H_ */

