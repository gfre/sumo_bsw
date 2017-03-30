/*******************************************************************************
 * @brief 	Tachometer implementation.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 		06.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * Module to calculate the speed based on the quadrature counter.
 *
 * ==============================================================================
 */

#ifndef __TACHO_H_
#define __TACHO_H_

#include "Platform.h"


#define TACHO_SWC_STRING ("tacho")

/*!
 * @brief Returns the previously calculated speed of the motor.
 * @param isLeft TRUE for left speed, FALSE for right speed.
 * @return Actual speed value
 */
int32_t TACHO_GetSpeed(bool isLeft);

/*!
 * @brief Calculates the speed based on the position information from the encoder.
 */
void TACHO_CalcSpeed(void);

/*!
 * @brief Sampling routine to calculate speed, must be called periodically with a fixed frequency.
 */
void TACHO_Sample(void);

#include "CLS1.h"
/*!
 * @brief Parses a command
 * @param cmd Command string to be parsed
 * @param handled Sets this variable to TRUE if command was handled
 * @param io I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
uint8_t TACHO_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);

/*! @brief De-initialization of the module */
void TACHO_Deinit(void);

/*! @brief Initialization of the module */
void TACHO_Init(void);


#endif /* __TACHO_H_ */
