/*******************************************************************************
 * @brief 	Module for the battery management.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date		09.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * Deals with the robot battery.
 *
 * ==============================================================================
 */

#ifndef BATT_H_
#define BATT_H_

#include "Platform.h"
#include "CLS1.h"
/*!
 * @brief Parses a command
 * @param cmd Command string to be parsed
 * @param handled Sets this variable to TRUE if command was handled
 * @param io I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
uint8_t BATT_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


/*!
 * @brief Does a measurement of the battery voltage
 * @param cvP Pointer to variable where to store the voltage in centi-voltage units (330 is 3.3V)
 * @return Error code, ERR_OK if everything was fine
 */
uint8_t BATT_MeasureBatteryVoltage(uint16_t *cvP);


/*!
 * @brief Module Initialization.
 */
void BATT_Init(void);

/*!
 * @brief Module De-initialization.
 */
void BATT_Deinit(void);



#endif /* BATT_H_ */
