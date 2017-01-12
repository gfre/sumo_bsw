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
#include "CLS1.h"

/*!
 * @brief Parses a command
 * @param cmd Command string to be parsed
 * @param handled Sets this variable to TRUE if command was handled
 * @param io I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
uint8_t DRV_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);

typedef enum {
  DRV_MODE_NONE,
  DRV_MODE_STOP,
  DRV_MODE_SPEED,
  DRV_MODE_POS,
} DRV_Mode;

uint8_t DRV_SetSpeed(int32_t left, int32_t right);
uint8_t DRV_SetPos(int32_t left, int32_t right);
bool DRV_IsDrivingBackward(void);
uint8_t DRV_SetMode(DRV_Mode mode);
DRV_Mode DRV_GetMode(void);
bool DRV_IsStopped(void);
bool DRV_HasTurned(void);

/*!
 * @brief Stops the engines
 * @param timoutMs timout in milliseconds for operation
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

#endif /* DRIVE_H_ */
