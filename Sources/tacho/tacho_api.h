/***********************************************************************************************//**
 * @file		tacho_api.h
 * @ingroup		tacho
 * @brief 		API of the SWC @a Tacho
 *
 * This API provides a BSW-internal interface of the SWC @ref tacho. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	04.05.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef TACHO_API_H_
#define TACHO_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"
#include "ACon_Types.h"



#ifdef MASTER_tacho_api_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup tacho
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the SWC @ref tacho
 */
#define TACHO_SWC_STRING ("Tacho")

/**
 *  Speed sample period in ms. Make sure that speed is sampled at the given rate.
 */
#define TACHO_SAMPLE_PERIOD_MS (5)

/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef void FltrFct_t(void);

typedef enum TACHO_FilterType_e{
	MOVING_AVERAGE_FILTER = 0,
	KALMAN_FILTER,
	TRACKING_LOOP_FILTER,
}TACHO_FltrType_t;

typedef struct TACHO_Fltr_s{
    char_t           *pFilterName;
    TACHO_FltrType_t FilterType;
	bool 		isInitialized;
	bool 		isUsingSampledSpeed;
    FltrFct_t 	*pFilterInitFct;
    FltrFct_t 	*pFilterMainFct;
    FltrFct_t   *pFilterDeinitFct;
	int32_t    (*pGetSpeedFct)(bool isLeft_);
}TACHO_Fltr_t;

typedef struct TACHO_Cfg_s{
	TACHO_Fltr_t* pFilterTable;
	int8_t		  NumOfFilters; 			// Anzahl an Filterkomponenten in Filtertable
}TACHO_Cfg_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
 /**
 * @brief Returns the previously calculated speed of the motor.
 * @param isLeft TRUE for left speed, FALSE for right speed.
 * @return Actual speed value
 */
EXTERNAL_ TACHO_Cfg_t* Get_pTachoCfg(void);

EXTERNAL_ StdRtn_t TACHO_Read_CurLftPos(int32_t* result_);
EXTERNAL_ StdRtn_t TACHO_Read_CurRghtPos(int32_t* result_);
EXTERNAL_ StdRtn_t TACHO_Read_CurUnfltrdLftSpd(int32_t* result_);
EXTERNAL_ StdRtn_t TACHO_Read_CurUnfltrdRghtSpd(int32_t* result_);
EXTERNAL_ StdRtn_t TACHO_Read_CurFltrdLftSpd(int32_t* result_);
EXTERNAL_ StdRtn_t TACHO_Read_CurFltrdRghtSpd(int32_t* result_);

EXTERNAL_ void TACHO_Set_FltrType(TACHO_FltrType_t type_);
EXTERNAL_ TACHO_FltrType_t TACHO_Get_FltrType(void);





/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_API_H_ */
