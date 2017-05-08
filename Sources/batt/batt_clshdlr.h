/***********************************************************************************************//**
 * @file		batt_clshdlr.h
 * @ingroup		batt
 * @brief 		Header for the command line shell handler of the SWC @a Battery
 *
 * This header files provides the interface from the SWC @ref batt to the SWC @ref sh.
 * It introduces application specific commands for requesting battery voltage information
 * via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	10.04.2017
 *
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef BATT_CLSHDLR_H_
#define BATT_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_batt_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup batt
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief This function parses a command of the SWC Battery for the CLS
 * @param cmd_ Command string to be parsed
 * @param handled_ Sets this variable to TRUE if command was handled
 * @param io_ I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
EXTERNAL_ uint8_t BATT_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !BATT_CLSHDLR_H_ */
