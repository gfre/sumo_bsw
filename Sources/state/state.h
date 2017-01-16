#ifndef STATE_H_
#define STATE_H_

#include "Platform.h"

#ifdef MASTER_STATE_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

typedef enum mainState_e{
	 MAIN_STATE_STARTUP
	,MAIN_STATE_INIT
	,MAIN_STATE_IDLE
	,MAIN_STATE_NORMAL
	,MAIN_STATE_ERROR
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
