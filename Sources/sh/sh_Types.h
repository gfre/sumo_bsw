/***************************************************************************************************
 * @brief 	This is an interface of the shell software component.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, University Kiel
 * @date 	09.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file sh_Types.h
 * 
 *==================================================================================================
 */


#ifndef SH_TYPES_H_
#define SH_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "CLS1.h"
#include "RTT1.h"

#ifdef MASTER_sh_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/*!
 * @brief Sends a string to the shell/console stdout
 * @param msg_ Zero terminated string to write
 */
#define SH_SENDSTR(msg_)                   ( CLS1_SendStr(msg_, CLS1_GetStdio()->stdOut) \
		                                   , CLS1_SendStr(msg_, RTT1_GetStdio()->stdOut) )
/*!
 * @brief Sends a error string to the shell/console stdout
 * @param msg_ Zero terminated string to write
 */
#define SH_SENDERRSTR(msg_)                ( CLS1_SendStr(msg_, CLS1_GetStdio()->stdErr) \
		                                   , CLS1_SendStr(msg_, RTT1_GetStdio()->stdErr) )

#define SH_FPRINTF(stream_, fmt_, args_)   ( XF1_xvformat(CLS1_printfPutChar, RTT1_GetStdio()->stream_, fmt_, args_) \
							               & XF1_xvformat(CLS1_printfPutChar, CLS1_GetStdio()->stream_, fmt_, args_) )

#define SH_PRINTF(fmt_, args_)             ( XF1_xvformat(CLS1_printfPutChar, RTT1_GetStdio()->stdOut, fmt_, args_) \
							               & XF1_xvformat(CLS1_printfPutChar, CLS1_GetStdio()->stdOut, fmt_, args_) )

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !SH_TYPES_H_ */
