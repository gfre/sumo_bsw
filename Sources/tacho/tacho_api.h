/***********************************************************************************************//**
 * @file		tacho_api.h
 * @ingroup		tacho
 * @brief 		API of the SWC @a Tacho
 *
 * This API provides a BSW-internal interface of the SWC @ref tacho. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author  S. Helling,      stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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
#include "ACon_Types.h"



#ifdef MASTER_tacho_api_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


/*======================================= >> #DEFINES << =========================================*/
#define TACHO_OBJECT_STRING( _id )		( _id == TACHO_ID_LEFT ? ("tacho left") : (_id == TACHO_ID_RIGHT ? ("tacho right") : ("") ) )

/**
 *  Speed sample period in ms. Make sure that speed is sampled at the given rate.
 */
#define TACHO_SAMPLE_PERIOD_MS (5)

/**
 *
 */
#define TACHO_FILTER_ID_INVALID  (0xFF)

/**
 *
 */
#define TACHO_SPEED_VALUE_INVALID (0xFFFF)



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum TACHO_ID_e
{
	 TACHO_ID_LEFT = 0
	,TACHO_ID_RIGHT
	,TACHO_ID_CNT
} TACHO_ID_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
 /**
 * @brief Returns the position of the left track in [steps].
 * @param pos_ Pointer to the position variable
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TACHO_Read_PosLe(int32_t* pos_);

/**
 * @brief Returns the position of the right track in [steps].
 * @param pos_ Pointer to the position variable
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TACHO_Read_PosRi(int32_t* pos_);

/**
 * @brief Returns the unfiltered speed of the left track in [steps/second].
 * @param spd_ Pointer to the speed variable
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TACHO_Read_RawSpdLe(int16_t* spd_);

/**
 * @brief Returns the unfiltered speed of the right track in [steps/second].
 * @param spd_ Pointer to the speed variable
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TACHO_Read_RawSpdRi(int16_t* spd_);

/**
 * @brief Returns the filtered speed of the left track in [steps/second].
 * @param spd_ Pointer to the speed variable
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TACHO_Read_SpdLe(int16_t* spd_);

/**
 * @brief Returns the filtered speed of the right track in [steps/second].
 * @param spd_ Pointer to the speed variable
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TACHO_Read_SpdRi(int16_t* spd_);

/**
 * @brief Changes the filter type
 * @param idx_ ID that corresponds to a @ref TACHO_FltrItm_t item
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t TACHO_Set_FltrReq(uint8_t idx_);

/**
 * @brief Returns the active filter ID
 * @return ID of active filter
 */
uint8_t TACHO_Get_ActFltrIdx(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_API_H_ */
