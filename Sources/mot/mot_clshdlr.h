/***********************************************************************************************//**
 * @file		mot_clshdlr.h
 * @ingroup		mot
 * @brief		Interface for the command line shell handler of the SWC @a Motor Driver
 *
 * This header files provides the interface from the SWC @ref mot to the SWC @ref sh.
 * It introduces application specific commands for debugging or requests of status information
 * via command line shell (@b CLS).
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	29.03.2017
 *  
 * @note Interface for CLS-specific use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef MOT_CLSHDLR_H_
#define MOT_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_motor_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup mot
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief Shell command line parser.
 * @param[in] cmd Pointer to command string
 * @param[out] handled If command is handled by the parser
 * @param[in] io Std I/O handler of shell
 */
EXTERNAL_ uint8_t MOT_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MOT_CLSHDLR_H_ */
