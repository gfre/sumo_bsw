/*******************************************************************************
 * @brief 	Shell and console interface implementation.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 	02.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements the front to the console/shell functionality.
 *
 * ==============================================================================
 */

#ifndef SH_H_
#define SH_H_

#include "CLS1.h"

/*!
 * @brief Sends a string to the shell/console stdout
 * @param msg Zero terminated string to write
 */
void SH_SendString(unsigned char *msg);

/*!
 * @brief Puts a command received from the Radio channel into a buffer.
 * @param str Zero terminated string
 */
void SH_RadioRxString(unsigned char *str);

/*!
 * @brief Parse a command string
 * @param cmd Zero terminated command to be parsed
 */
void SH_ParseCmd(unsigned char *cmd);

/*!
 * @brief Checks if there is input from the console and parses it.
 */
void SH_Parse(void);

CLS1_ConstStdIOType *SH_GetStdio(void);

/*! @brief Shell initialization */
void SH_Init(void);

/*! @brief Shell main function */
void SH_MainFct(void);

/*! @brief Serial driver de-initialization */
void SH_Deinit(void);

#endif /* SH_H_ */
