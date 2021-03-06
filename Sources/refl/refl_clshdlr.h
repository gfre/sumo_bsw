/***********************************************************************************************//**
 * @file		refl_clshdlr.h
 * @ingroup		refl
 * @brief 		Interface for the command line shell handler of the SWC @a RNet
 *
 * This header files provides the interface from the SWC @ref refl to the SWC @ref sh.
 * It introduces application specific commands for requests of status information
 * via command line shell (@b CLS)
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef REFL_CLSHDLR_H_
#define REFL_CLSHDLR_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"



#ifdef MASTER_refl_clshdlr_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


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
EXTERNAL_ uint8_t REFL_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !REFL_CLSHDLR_H_ */
