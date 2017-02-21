/***************************************************************************************************
 * @brief 	Driver for the buzzer.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, University Kiel
 * @date 	09.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
  *==================================================================================================
 */

#ifndef BUZ_H_
#define BUZ_H_

#include "Platform.h"
#include "CLS1.h"
#include "rte_Types.h"

/*!
 * @brief Shell parser routine.
 * @param cmd Pointer to command line string.
 * @param handled Pointer to status if command has been handled. Set to TRUE if command was understood.
 * @param io Pointer to stdio handle
 * @return Error code, ERR_OK if everything was ok.
 */
uint8_t BUZ_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


/*!
 * @brief Let the buzzer sound for a specified time.
 * @param freqHz Frequency of the sound. Ignored if the buzzer is not supporting it.
 * @param durationMs Duration in milliseconds.
 * @return Error code, ERR_OK if everything is fine.
 */
uint8_t BUZ_Beep(uint16_t freqHz, uint16_t durationMs);


/*!
 * @brief Plays a tune
 * @param tune Tune to play
 * @return ERR_OK or error code
 */
uint8_t BUZ_PlayTune(BUZ_Tunes_t tune);


/*!
 * @brief Initialization of the driver
 */
void BUZ_Init(void);


/*!
 * @brief De-initialization of the driver
 */
void BUZ_Deinit(void);


#endif /* BUZ_H_ */
