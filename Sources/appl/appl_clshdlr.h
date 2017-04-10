/***********************************************************************************************//**
 * @file		appl_clshdlr.h
 * @ingroup		appl
 * @brief		Header for the command line shell handler of the SWC @a Application
 *
 * This header files provides the interface to the command line shell (@b CLS). It introduces
 * application specific commands for debugging or gathering status information about the
 * application via CLS.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef APPL_CLSHDLR_H_
#define APPL_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_appl_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup <group label>
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Parses a command of the software component 'application'
 *
 * @param cmd_ Command string to be parsed
 * @param handled_ Sets this variable to TRUE if command was handled
 * @param io_ I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
EXTERNAL_ uint8 APPL_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !APPL_CLSHDLR_H_ */
