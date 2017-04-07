/*******************************************************************************
 * @brief 	Module to drive the robot.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 		06.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module allows to drive the robot and to perform turns.
 *
 * ==============================================================================
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#include "Platform.h"

#define DRV_SWC_STRING ("drive")


/*!
 * @brief Stops the engines
 * @param timeoutMs timout in milliseconds for operation
 * @return ERR_OK if stopped, ERR_BUSY for timeout condition.
 */
uint8_t DRV_Stop(int32_t timeoutMs);

/*!
 * @brief Driver initialization.
 */
void DRV_Init(void);

/*!
 * @brief Driver de-initialization.
 */
void DRV_DeInit(void);


void DRV_MainFct(void);

#endif /* DRIVE_H_ */
