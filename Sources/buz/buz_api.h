/***********************************************************************************************//**
 * @file		buz_api.h
 * @ingroup		buz
 * @brief 		API of the SWC @a Buzzer
 *
 * This API provides an internal interface of the Basic Software from the SWC @a Buzzer to the
 * all other Basic Software Components.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.04.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef BUZ_API_H_
#define BUZ_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "rte_Types.h"



#ifdef MASTER_buz_api_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup buz
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Let the buzzer sound for a specified time.
 * @param freqHz_ Frequency of the sound. Ignored if the buzzer is not supporting it.
 * @param durationMs_ Duration in milliseconds.
 * @return Error code, ERR_OK if everything is fine.
 */
EXTERNAL_ uint8_t BUZ_Beep(uint16_t freqHz_, uint16_t durationMs_);



/**
 * @brief Plays a tune
 * @param tune_ Tune to play
 * @return ERR_OK or error code
 */
EXTERNAL_ uint8_t BUZ_PlayTune(BUZ_Tunes_t tune_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !BUZ_API_H_ */
