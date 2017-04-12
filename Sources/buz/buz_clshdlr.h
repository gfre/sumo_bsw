/***********************************************************************************************//**
 * @file		buz_clshdlr.h
 * @ingroup		buz
 * @brief 		Interface for the command line shell handler of the SWC @a Buzzer
 *
 * This header files provides the interface from the SWC @ref buz which is addressed the SWC @ref sh.
 * It introduces application specific commands for requesting buzzer help and status information
 * via command line shell (@b CLS).
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

#ifndef BUZ_CLSHDLR_H_
#define BUZ_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_buz_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup buz
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Shell parser routine.
 * @param cmd_ Pointer to command line string.
 * @param handled_ Pointer to status if command has been handled. Set to TRUE if command was understood.
 * @param io_ Pointer to stdio handle
 * @return Error code, ERR_OK if everything was ok.
 */
EXTERNAL_ uint8_t BUZ_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !BUZ_CLSHDLR_H_ */
