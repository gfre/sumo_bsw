/***********************************************************************************************//**
 * @file		rnet_clshdlr.h
 * @ingroup		rnet
 * @brief		Interface for the command line shell handler of the SWC @a RNet
 *
 * This header files provides the interface from the SWC @ref rnet to the SWC @ref sh.
 * It introduces application specific commands for requests of status information
 * via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	17.02.2017
 *  
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef RNET_CLSHDLR_H_
#define RNET_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_rnet_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup rnet
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Parses a command of the software component 'RF network stack'
 * @param cmd Command string to be parsed
 * @param handled Sets this variable to TRUE if command was handled
 * @param io I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
EXTERNAL_ uint8_t RNET_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !RNET_CLSHDLR_H_ */
