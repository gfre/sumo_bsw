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



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef void FilterFct_t(void);

typedef enum TACHO_FilterType_e{
	MOVING_AVERAGE_FILTER = 0,
	KALMAN_FILTER,
	TRACKING_LOOP_FILTER,
}TACHO_FilterType_t;

typedef struct TACHO_Filter_s{
    char_t* pFilterName;
    TACHO_FilterType_t FilterType;
	bool isInitialized;
	bool isUsingSampledSpeed;
    FilterFct_t *pFilterInitFct;
    FilterFct_t *pFilterMainFct;
    FilterFct_t *pFilterDeinitFct;
	int32_t (*pGetSpeedFct)(bool isLeft_);
}TACHO_Filter_t;

typedef struct TACHO_Cfg_s{
	TACHO_Filter_t* pFilterTable;
	int8_t NumOfFilters; 			// Anzahl an Filterkomponenten in Filtertable
}TACHO_Cfg_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
 /**
 * @brief Returns the previously calculated speed of the motor.
 * @param isLeft TRUE for left speed, FALSE for right speed.
 * @return Actual speed value
 */
EXTERNAL_ TACHO_Cfg_t* Get_pTachoCfg(void);

EXTERNAL_ StdRtn_t TACHO_Read_CurLeftPos(int32_t* pos_);
EXTERNAL_ StdRtn_t TACHO_Read_CurRightPos(int32_t* pos_);
EXTERNAL_ StdRtn_t TACHO_Read_CurUnfltrdLeftSpd(int32_t* speed_);
EXTERNAL_ StdRtn_t TACHO_Read_CurUnfltrdRightSpd(int32_t* speed_);
EXTERNAL_ StdRtn_t TACHO_Read_CurFltrdLeftSpd(int32_t* speed_);
EXTERNAL_ StdRtn_t TACHO_Read_CurFltrdRightSpd(int32_t* speed_);

EXTERNAL_ void TACHO_Set_FilterType(TACHO_FilterType_t type_);
EXTERNAL_ TACHO_FilterType_t TACHO_Get_FilterType(void);





/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_API_H_ */
