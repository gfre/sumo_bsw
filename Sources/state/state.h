/*******************************************************************************
 * @brief 	Main state machine of application software layer.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date 	13.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements an (extended) state machine for the Sumo robot. It
 * handles the main states of the application software layer.
 *
 * ==============================================================================
 */

#ifndef STATE_H_
#define STATE_H_

#include "Platform.h"

#ifdef MASTER_STATE_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

typedef enum mainState_e{
	 MAIN_STATE_STARTUP     /**< State during start up */
	,MAIN_STATE_INIT        /**< State during initialization */
	,MAIN_STATE_IDLE        /**< State for idle mode */
	,MAIN_STATE_NORMAL		/**< State for normal mode */
	,MAIN_STATE_ERROR       /**< State for error mode */
} mainState_t;

/*!
 * @brief Main function of the software component 'state'
 * Runs the state machine of the main state
 */
EXTERNAL_ void STATE_mainFct(void);

/*!
 * @brief Parses a command of the software component 'state'
 * @param cmd Command string to be parsed
 * @param handled Sets this variable to TRUE if command was handled
 * @param io I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
EXTERNAL_ uint8 STATE_ParseCommand(const unsigned char *cmd_, bool *handled, const CLS1_StdIOType *io_);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !STATE_H_ */
