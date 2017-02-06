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

#ifdef MASTER_SH_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

#define SH_SWC_STRING ("shell")
#define SH_CMD_EXIT   ("exit")


/*!
 * @brief Sends a string to the shell/console stdout
 * @param msg_ Zero terminated string to write
 */
EXTERNAL_ void SH_SendStr(unsigned char *msg_);

/*!
 * @brief Sends a error string to the shell/console stdout
 * @param msg_ Zero terminated string to write
 */
EXTERNAL_ void SH_SendErrStr(unsigned char *msg_);

/*!
 * @brief Sends a error string to the shell/console stdout
 * @param *cmd_ pointer to command
 * @param *handeld_ pointer to flag which returns TRUE if cmd was handeld, otherwise FALSE (CBR).
 * @param *io_ pointer to command line shell standard IO type
 * @return always ERR_OK
 */
EXTERNAL_ uint8_t SH_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);

/*! @brief Shell initialization */
EXTERNAL_ void SH_Init(void);

/*! @brief Shell main function */
EXTERNAL_ void SH_MainFct(void);

/*! @brief Serial driver de-initialization */
EXTERNAL_ void SH_Deinit(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !SH_H_ */
