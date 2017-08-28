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
#define TACHO_MAX_NUM_OF_FILTERS (0xFA)


/*=================================== >> TYPE DEFINITIONS << =====================================*/

/**
 *
 */
typedef void FltrRtFct_t(void);

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
    bool reqRawSpd;
	FltrRtFct_t *initFct;
	FltrRtFct_t *mainFct;
	FltrRtFct_t *deinitFct;
	FltrRtFct_t *sampleCbFct;
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


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_CFG_H_ */
