/***********************************************************************************************//**
 * @file		tacho_clshdlr.h
 * @ingroup		tacho
 * @brief		Interface for the command line shell handler of the SWC @a Tacho
 *
 * This header files provides the interface from the SWC @ref tacho to the SWC @ref sh.
 * It introduces application specific commands for requests of status information
 * via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	30.03.2017
 *  
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef TACHO_CLSHDLR_H_
#define TACHO_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_tacho_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Parses a command
 * @param cmd_ Command string to be parsed
 * @param handled_ Sets this variable to TRUE if command was handled
 * @param io_ I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
uint8_t TACHO_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TACHO_CLSHDLR_H_ */
