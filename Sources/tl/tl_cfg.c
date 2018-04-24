/***********************************************************************************************//**
 * @file		tl_cfg.c
 * @ingroup		tl
 * @brief 		Filter component to estimate system state using a PI tracking loop
 *
 * This file implements the configuration of tracking loop filters, which which implements an
 * estimation algorithm for dynamical system with two states and an internal interface for the
 * tracking loop configuration.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author 	S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	17.08.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_tl_cfg_C_

//TODO
// - decouple sample time from calling task period and runtime calls
// - Connect PID gain cfg to NVM
// - Adapt cls handler similar to handler of PID

/*======================================= >> #INCLUDES << ========================================*/
#include "tl_cfg.h"

/** Application APIs */
#include "tacho_api.h"



/*======================================= >> #DEFINES << =========================================*/
/**
 * Default saturation value for the underlying PID controller
 */
#define TL_DFLT_PID_SATURATION_VALUE	(0xFFFFu)

/**
 * Initialisation of runtime data
 */
#define TL_DFLT_DATA_INIT  				{0}

/**
 * Default D-gain for the PID controller must be zero for PI control only
 */
#define TL_DFLT_D_GAIN					(0x0u)



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static TL_Itm_t items[] =
{
		{	{TACHO_OBJECT_STRING(TACHO_ID_LEFT),  TACHO_SAMPLE_PERIOD_MS, {3500u, 25u, TL_DFLT_D_GAIN, 100u, TL_DFLT_PID_SATURATION_VALUE},
			TACHO_Read_PosLe},  TL_DFLT_DATA_INIT
		},
		{ 	{TACHO_OBJECT_STRING(TACHO_ID_RIGHT), TACHO_SAMPLE_PERIOD_MS, {3500u, 25u, TL_DFLT_D_GAIN, 100u, TL_DFLT_PID_SATURATION_VALUE},
			TACHO_Read_PosRi}, TL_DFLT_DATA_INIT
		},
};

static TL_ItmTbl_t itemTable =
{
		items,
		sizeof(items)/(sizeof(items[0])),
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
TL_ItmTbl_t *Get_pTlItmTbl(void) {return &itemTable;}



#ifdef MASTER_tl_cfg_C_
#undef MASTER_tl_cfg_C_
#endif /* !MASTER_tl_cfg_C_ */
