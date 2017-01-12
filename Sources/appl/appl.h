/*******************************************************************************
 * @brief 	Main Application Interface.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date		02.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * Interface to the main application module. From here the application
 * runs and performs all tasks.
 *
 * ==============================================================================
 */

#ifndef APPL_H_
#define APPL_H_

#include "CLS1.h"

/*!
 * @brief Debug printing function
 * @param str Debug string to print
 */
void APP_DebugPrint(unsigned char *str);

/*!
 * @brief Parses a command
 * @param cmd Command string to be parsed
 * @param handled Sets this variable to TRUE if command was handled
 * @param io I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
uint8_t APP_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);


/*!
 * @brief Run the application
 */
void APP_Run(void);

#endif /* APPLICATION_H_ */
