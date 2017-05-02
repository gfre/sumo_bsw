/***********************************************************************************************//**
 * @file		id_clshdlr.h
 * @ingroup		id
 * @brief 		Interface for the command line shell handler of the SWC @a Shell
 *
 * This header files provides the internal interface of the SWC @ref sh  which allows to debug the
 * overlaying @a shell component by itself. It introduces application specific commands for requests
 * of status information via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.02.2017
 *
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef SH_CLSHDLR_H_
#define SH_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_sh_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup sh
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Sends a error string to the shell/console stdout
 * @param *cmd_ pointer to command
 * @param *handled_ pointer to flag which returns TRUE if cmd_ was handled_, otherwise FALSE (CBR).
 * @param *io_ pointer to command line shell standard IO type
 * @return always ERR_OK
 */
EXTERNAL_ uint8_t SH_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !SH_CLSHDLR_H_ */
