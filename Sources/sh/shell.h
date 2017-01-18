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

#ifndef SHELL_H_
#define SHELL_H_

#include "CLS1.h"

/*!
 * @brief Sends a string to the shell/console stdout
 * @param msg Zero terminated string to write
 */
void SHELL_SendString(unsigned char *msg);

/*!
 * @brief Puts a command received from the Radio channel into a buffer.
 * @param str Zero terminated string
 */
void SHELL_RadioRxString(unsigned char *str);

/*!
 * @brief Parse a command string
 * @param cmd Zero terminated command to be parsed
 */
void SHELL_ParseCmd(unsigned char *cmd);

/*!
 * @brief Checks if there is input from the console and parses it.
 */
void SHELL_Parse(void);

CLS1_ConstStdIOType *SHELL_GetStdio(void);

/*! @brief Shell initialization */
void SHELL_Init(void);

/*! @brief Shell main function */
void SHELL_MainFct(void);

/*! @brief Serial driver de-initialization */
void SHELL_Deinit(void);

#endif /* SHELL_H_ */
