/***********************************************************************************************//**
 * @file		refl.h
 * @ingroup		refl
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	Simon Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef SOURCES_REFL_H_
#define SOURCES_REFL_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


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
/*/**
 * @brief
 *
 */
#define REF_MIN_LINE_VAL      0x120   /* minimum value indicating a line */
//#define REF_MIN_NOISE_VAL      0x40   /* values below this are not added to the weighted sum */
#define REF_MIN_NOISE_VAL      0x80   /* values below this are not added to the weighted sum */
#define REF_SENSOR_TIMEOUT_US  3500   /* after this time, consider no reflection (black). Must be smaller than the timeout period of the RefCnt timer! */
#define REF_NOF_SENSORS 6
#define REF_MIDDLE_LINE_VALUE  ((REF_NOF_SENSORS+1)*1000/2)
#define REF_MAX_LINE_VALUE     ((REF_NOF_SENSORS+1)*1000) /* maximum value for REF_GetLine() */


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
	

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * \brief Shell parser routine.
 * \param cmd Pointer to command line string.
 * \param handled Pointer to status if command has been handled. Set to TRUE if command was understood.
 * \param io Pointer to stdio handle
 * \return Error code, ERR_OK if everything was ok.
 */
EXTERNAL_ uint8_t REF_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);

EXTERNAL_ REF_LineKind REF_GetLineKind(void);

EXTERNAL_ void REF_DumpCalibrated(void);

EXTERNAL_ unsigned char *REF_LineKindStr(REF_LineKind line);

EXTERNAL_ uint16_t REF_GetLineValue(bool *onLine);

EXTERNAL_ uint16_t REF_LineWidth(void);

void REF_GetSensorValues(uint16_t *values, int nofValues);

/*!
 * \brief Starts or stops the calibration.
 */
void REF_CalibrateStartStop(void);

/*!
 * \brief Function to find out if we can use the sensor (means: it is calibrated and not currently calibrating)
 * \return TRUE if the sensor is ready.
 */
bool REF_CanUseSensor(void);

/*!
 * \brief Driver Deinitialization.
 */
void REF_Deinit(void);

/*!
 * \brief Driver Initialization.
 */
void REF_Init(void);


/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !SOURCES_REFL_H_ */
