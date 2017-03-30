/***************************************************************************************************
 * @brief 	Implementation of PID controllers.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 		06.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements PID controllers for position and speed control of the sumo robots. It uses
 * the NVM software component for storing the controller parameters.
 *
 *==================================================================================================
 */


#ifndef PID_H_
#define PID_H_

/*======================================= >> #INCLUDES << ========================================*/


#ifdef MASTER_pid_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum {
  PID_CONFIG_LINE_FW,
  PID_CONFIG_LINE_BW,
  PID_CONFIG_POS_LEFT,
  PID_CONFIG_POS_RIGHT,
  PID_CONFIG_SPEED_LEFT,
  PID_CONFIG_SPEED_RIGHT
} PID_ConfigType;

typedef struct {
  int32_t pFactor100;
  int32_t iFactor100;
  int32_t dFactor100;
  int32_t iAntiWindup;
  uint8_t maxSpeedPercent; /* max speed if 100% on the line, 0xffff would be full speed */
  int32_t lastError;
  int32_t integral;
} PID_Config;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Performs PID on a line
 * @param currLinePos Current line position
 * @param setLinePos Desired line position
 * @param currLineWidth Indication of line width (in 1000er units for a line)
 * @param forward If we are moving forward or backward
 */
void PID_Line(uint16_t currLinePos, uint16_t setLinePos, uint16_t currLineWidth, bool forward);

/*!
 * @brief Performs PID closed loop calculation for the speed
 * @param currSpeed Current speed of motor
 * @param setSpeed desired speed of motor
 * @param isLeft TRUE if is for the left motor, otherwise for the right motor
 */
void PID_Speed(int32_t currSpeed, int32_t setSpeed, bool isLeft);

/*!
 * @brief Performs PID closed loop calculation for the line position
 * @param currPos Current position of wheel
 * @param setPos Desired wheel position
 * @param isLeft TRUE if is for the left wheel, otherwise for the right wheel
 */
void PID_Pos(int32_t currPos, int32_t setPos, bool isLeft);

/*! @brief Driver initialization */
void PID_Start(void);

/*! @brief Driver initialization */
void PID_Init(void);

/*! @brief Driver de-initialization */
void PID_Deinit(void);

/*!
 * @brief Function returns PID parameter configuration for position control on the left hand side
 * @return PID parameter configuration
 */
PID_Config *PID_Get_PosLeCfg(void);

/*!
 * @brief Function returns PID parameter configuration for position control on the right hand side
 * @return PID parameter configuration
 */
PID_Config *PID_Get_PosRiCfg(void);

/*!
 * @brief Function returns PID parameter configuration for speed control on the left hand side
 * @return PID parameter configuration
 */
PID_Config *PID_Get_SpdLeCfg(void);

/*!
 * @brief Function returns PID parameter configuration for speed control on the right hand side
 * @return PID parameter configuration
 */
PID_Config *PID_Get_SpdRiCfg(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* PID_H_ */
