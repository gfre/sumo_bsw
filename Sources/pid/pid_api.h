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
typedef  StdRtn_t ReadValFct_t(int32_t*);

typedef enum PID_PlantType_e
{
	PID_LEFT_MOTOR_SPEED = 0,
	PID_RIGHT_MOTOR_SPEED,
	PID_LEFT_MOTOR_POS,
	PID_RIGHT_MOTOR_POS,
}PID_PlantType_t;

typedef enum PID_SatType_e
{
	PID_NEG_SAT = -1,
	PID_NO_SAT,
	PID_POS_SAT,
}PID_SatType_t;

typedef struct PID_Cfg_s
{
	uint32_t Factor_KP_scld;
	uint32_t Factor_KI_scld;
	uint32_t Factor_KD_scld;
	uint8_t  Scale;
	uint32_t  iWindUpMaxVal;
}PID_Cfg_t;

typedef struct PID_Plant_s
{
	char_t* 		pPlantName;
	PID_PlantType_t PlantType;
	PID_Cfg_t*      Config;
	PID_SatType_t	Saturation;
	int32_t 		lastError;
	int32_t 		integralVal;
	ReadValFct_t    *pCurValFct;
	ReadValFct_t    *pTrgtValFct;
}PID_Plant_t;

typedef struct PID_PlantCfg_s
{
	PID_Plant_t* pPlantTbl;
	int8_t       numOfPlants;
}PID_PlantCfg_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/

EXTERNAL_ StdRtn_t PI(PID_Plant_t* plant_, int32_t*);

EXTERNAL_ PID_PlantCfg_t* Get_pPidCfg(void);

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !PID_API_H_ */
