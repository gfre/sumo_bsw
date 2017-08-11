/***********************************************************************************************//**
 * @file		pid_api.h
 * @ingroup		pid
 * @brief 		API of the SWC @a PID
 *
 * This API provides a BSW-internal interface of the SWC @ref pid. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	27.04.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef PID_API_H_
#define PID_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"
#include "Platform.h"

#ifdef MASTER_pid_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup pid
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @typedef PID_Config_t
 * @brief Data type definition of the structure PID_Config_s
 *
 * @struct PID_Config_s
 * @brief This structure defines the parameters of a [PID controller](@ref pid) and its current data.
 */
typedef struct PID_Config_s {
  int32_t pFactor100;
  int32_t iFactor100;
  int32_t dFactor100;
  int32_t iAntiWindup;
  uint8_t maxSpeedPercent;			/**< max speed if 100% on the line, 0xffff would be full speed */
  int32_t lastError;
  int32_t integral;
} PID_Config_t;
	

typedef enum PID_PlantType_e
{
	PID_LEFT_MOTOR_SPEED = 0,
	PID_RIGHT_MOTOR_SPEED,
	PID_LEFT_MOTOR_POS,
	PID_RIGHT_MOTOR_POS,
}PID_PlantType_t;

typedef struct PID_Plant_s
{
	PID_PlantType_t PlantType;
	bool isInitialized;
	int32_t Factor_KP_scld;
	int32_t Factor_KI_scld;
	int32_t Factor_KD_scld;
	int32_t Scale;
	int32_t Saturation;
	int32_t iWindUpMaxVal;
	int32_t lastError;
	int32_t integralVal;
	void (*pInitFct)(PID_Plant_t);
	void (*pDeinitFct)(PID_Plant_t);
	int32_t (*pGetCurrentValFct)(bool);
	int32_t (*pGetTargetValFct)(void);
}PID_Plant_t;

typedef struct PID_PlantCfg_s
{
	PID_Plant_t* pPlantTbl;
	int8_t       numOfPlants;
}PID_PlantCfg_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Performs PID on a line
 * @param currLinePos Current line position
 * @param setLinePos Desired line position
 * @param currLineWidth Indication of line width (in 1000er units for a line)
 * @param forward If we are moving forward or backward
 */
EXTERNAL_ void PID_Line(uint16_t currLinePos, uint16_t setLinePos, uint16_t currLineWidth, bool forward);

/**
 * @brief Performs PID closed loop calculation for the speed
 * @param currSpeed Current speed of motor
 * @param setSpeed desired speed of motor
 * @param isLeft TRUE if is for the left motor, otherwise for the right motor
 */
EXTERNAL_ void PID_Speed(int32_t currSpeed, int32_t setSpeed, bool isLeft);

/**
 * @brief Performs PID closed loop calculation for the line position
 * @param currPos Current position of wheel
 * @param setPos Desired wheel position
 * @param isLeft TRUE if is for the left wheel, otherwise for the right wheel
 */
EXTERNAL_ void PID_Pos(int32_t currPos, int32_t setPos, bool isLeft);

/**
 * @brief Driver initialization
 */
EXTERNAL_ void PID_Start(void);

/**
 * @brief Function returns PID parameter configuration for position control on the left hand side
 * @return PID parameter configuration
 */
EXTERNAL_ PID_Config_t *PID_Get_PosLeCfg(void);

/**
 * @brief Function returns PID parameter configuration for position control on the right hand side
 * @return PID parameter configuration
 */
EXTERNAL_ PID_Config_t *PID_Get_PosRiCfg(void);

/**
 * @brief Function returns PID parameter configuration for speed control on the left hand side
 * @return PID parameter configuration
 */
EXTERNAL_ PID_Config_t *PID_Get_SpdLeCfg(void);

/**
 * @brief Function returns PID parameter configuration for speed control on the right hand side
 * @return PID parameter configuration
 */
EXTERNAL_ PID_Config_t *PID_Get_SpdRiCfg(void);

EXTERNAL_ PID_PlantCfg_t* Get_pPidCfg(void);

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !PID_API_H_ */
