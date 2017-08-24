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
#include "tacho_api.h"


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

/**
 *
 */
typedef void FltrFct_t(void);

/**
 *
 * @param
 * @return
 */
typedef int32_t ApiFct_t(bool);

/**
 *
 */
typedef struct TACHO_FltrItm_s
{
    const uchar_t *aFltrName;
    TACHO_Fltr_t fltrType;
	bool reqUnfltrdSpd;
    FltrFct_t *initFct;
    FltrFct_t *mainFct;
    FltrFct_t *deinitFct;
    ApiFct_t *apiSpeedFct;
} TACHO_FltrItm_t;

/**
 *
 */
typedef struct TACHO_FltrItmTbl_s
{
	TACHO_FltrItm_t *aFltrs;
	uint8_t numFltrs;
} TACHO_FltrItmTbl_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ TACHO_FltrItmTbl_t* Get_pFltrTbl(void);
/**
 *
 * @param type_
 * @return
 */
EXTERNAL_ StdRtn_t TACHO_Req_FilterType(TACHO_Fltr_t type_);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_CFG_H_ */
