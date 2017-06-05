/***********************************************************************************************//**
 * @file		refl_api.h
 * @ingroup		refl
 * @brief 		API of the SWC @a Reflectance Sensor Array
 *
 * This API provides a BSW-internal interface of the SWC @ref refl. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	02.06.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#ifndef REFL_API_H_
#define REFL_API_H_

/*======================================= >> #INCLUDES << ========================================*/



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



/*=================================== >> TYPE DEFINITIONS << =====================================*/

typedef enum {
  REF_LINE_NONE=0,     /* no line, sensors do not see a line */
  REF_LINE_STRAIGHT=1, /* forward line |, sensors see a line underneath */
  REF_LINE_LEFT=2,     /* left half of sensors see line */
  REF_LINE_RIGHT=3,    /* right half of sensors see line */
  REF_LINE_FULL=4,     /* all sensors see a line */
  REF_LINE_AIR=5,      /* all sensors have a timeout value. Robot is not on ground at all? */
  REF_NOF_LINES        /* Sentinel */
} REF_LineKind;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/

REF_LineKind REF_GetLineKind(void);

void REF_DumpCalibrated(void);

unsigned char *REF_LineKindStr(REF_LineKind line);

uint16_t REF_GetLineValue(bool *onLine);

uint16_t REF_LineWidth(void);

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

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !REFL_API_H_ */
