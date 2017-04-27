/***********************************************************************************************//**
 * @file		nvm_clshdlr.h
 * @ingroup		nvm
 * @brief		Interface for the command line shell handler of the SWC @a NVM
 *
 * This header files provides the interface from the SWC @ref nvm to the SWC @ref sh.
 * It introduces application specific commands for requests of status information or
 * restoring the NVM from ROM via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	23.02.2017
 *  
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/


#ifndef NVM_CLSHDLR_H_
#define NVM_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


#ifdef MASTER_nvm_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup nvm
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Parses a command of the software component 'non-volatile memory'
 * @param cmd Command string to be parsed
 * @param handled Sets this variable to TRUE if command was handled
 * @param io I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
EXTERNAL_ uint8_t NVM_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !NVM_CLSHDLR_H_ */
