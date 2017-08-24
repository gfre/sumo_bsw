/***********************************************************************************************//**
 * @file		tacho_cfg.h
 * @ingroup		tacho
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	13.06.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef TACHO_CFG_H_
#define TACHO_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"



#ifdef MASTER_tacho_cfg_C_
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
typedef void FltrFct_t(void);

typedef enum TACHO_FilterType_e{
	MOVING_AVERAGE_FILTER = 0,
	KALMAN_FILTER,
	TRACKING_LOOP_FILTER,
}TACHO_FltrType_t;

typedef struct TACHO_Fltr_s{
    char_t           *pFilterName;
    TACHO_FltrType_t FilterType;
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
EXTERNAL_ TACHO_Cfg_t* Get_pTachoCfg(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_CFG_H_ */
