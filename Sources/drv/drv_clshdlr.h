/***********************************************************************************************//**
 * @file		drv_clshdlr.h
 * @ingroup		drv
 * @brief 		Interface for the command line shell handler of the SWC @a Drive
 *
 * This header files provides the interface of the SWC @ref drv which is adressed to the SWC @ref sh.
 * It introduces application specific commands for requesting drive related help and status
 * information via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.04.2017
 *
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/


#ifndef DRV_CLSHDLR_H_
#define DRV_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"


#ifdef MASTER_drv_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup drv
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Parses a command
 * @param cmd_ Command string to be parsed
 * @param handled_ Sets this variable to TRUE if command was handled
 * @param io_ I/O stream to be used for input/output
 * @return Error code, ERR_OK if everything was fine
 */
uint8_t DRV_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !DRV_CLSHDLR_H_ */
