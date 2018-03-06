/***********************************************************************************************//**
 * @file		tl_clshdlr.h
 * @ingroup		tl
 * @brief		Interface for the command line shell handler of the SWC @a TL
 *
 * This header files provides the interface from the SWC @ref tl to the SWC @ref sh.
 * It introduces application specific commands for requests of status and help information.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	18.08.2017
 *  
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef TL_CLSHDLR_H_
#define TL_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_tl_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup tl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Shell command line parser.
 * @param cmd_ Pointer to command string
 * @param handled_ If command is handled by the parser
 * @param io_ Std I/O handler of shell
 */
EXTERNAL_ uint8_t  TL_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !TL_CLSHDLR_H_ */
