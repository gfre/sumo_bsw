/***********************************************************************************************//**
 * @file		sh_api.h
 * @ingroup		sh
 * @brief 		API of the SWC @a Shell
 *
 * This API provides a BSW-internal interface of the SWC @ref sh. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.02.2017
 *  
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef SH_APIH_
#define SH_APIH_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"
#include "RTT1.h"



#ifdef MASTER_sh_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup sh
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * @brief Sends a string to the shell/console stdout
 * @param msg_ Zero terminated string to write
 */
#define SH_SENDSTR(msg_)                   ( CLS1_SendStr(msg_, CLS1_GetStdio()->stdOut) \
		                                   , CLS1_SendStr(msg_, RTT1_GetStdio()->stdOut) )
/**
 * @brief Sends a error string to the shell/console stderr
 * @param msg_ Zero terminated string to write
 */
#define SH_SENDERRSTR(msg_)                ( CLS1_SendStr(msg_, CLS1_GetStdio()->stdErr) \
		                                   , CLS1_SendStr(msg_, RTT1_GetStdio()->stdErr) )
/**
 * @brief Sends a formatted string to the a user-defined I/O stream
 * @param stream_ user-defined I/O stream
 * @param fmt_ formatted string to write
 * @param args_ arguments within string
 */
#define SH_FPRINTF(stream_, fmt_, args_)   ( XF1_xvformat(CLS1_printfPutChar, RTT1_GetStdio()->stream_, fmt_, args_) \
							               & XF1_xvformat(CLS1_printfPutChar, CLS1_GetStdio()->stream_, fmt_, args_) )
/**
 * @brief Sends a formatted string to the shell/console stdout
 * @param fmt_ formatted string to write
 * @param args_ arguments within string
 */
#define SH_PRINTF(fmt_, args_)             ( XF1_xvformat(CLS1_printfPutChar, RTT1_GetStdio()->stdOut, fmt_, args_) \
							               & XF1_xvformat(CLS1_printfPutChar, CLS1_GetStdio()->stdOut, fmt_, args_) )

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !SH_APIH_ */
