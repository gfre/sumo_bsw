/*******************************************************************************
 * @brief 	Driver for the buzzer.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 		09.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include "Platform.h"
#include "CLS1.h"

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

typedef enum {
	BUZ_TUNE_WELCOME,
	BUZ_TUNE_BUTTON,
	BUZ_TUNE_BUTTON_LONG,
	BUZ_TUNE_NOF_TUNES
} BUZ_Tunes;

/*!
 * @brief Plays a tune
 * @param tune Tune to play
 * @return ERR_OK or error code
 */
uint8_t BUZ_PlayTune(BUZ_Tunes tune);

/*!
 * @brief Initialization of the driver
 */
void BUZ_Init(void);

/*!
 * @brief De-initialization of the driver
 */
void BUZ_Deinit(void);


#endif /* BUZZER_H_ */
