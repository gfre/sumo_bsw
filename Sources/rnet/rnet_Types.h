/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	17.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file rnet_Types.h
 * 
 *==================================================================================================
 */


#ifndef RNET_TYPES_H_
#define RNET_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "RApp.h"


#ifdef MASTER_RNET_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Fucntion returns the current network address of the RF-node.
 * @return network address
 */
EXTERNAL_ RAPP_ShortAddrType RNET_GetDstAddr(void);

/**
 * @brief Function sets the network address of the RF-node.
 * @param addr_ network address
 * @return void
 */
EXTERNAL_ void RNET_SetDstAddr(RAPP_ShortAddrType addr_);

/**
 * @brief Function returns the pointer to the callback function which handles receiving message
 * for the RTE layer.
 * @return pointer to the callback function
 */
EXTERNAL_ RAPP_RxMsg_CbFct *RNET_GetRTERxMsgCbFct(void);

/**
 * @brief Function sets the pointer to the callback function which handles receiving messages
 * for the RTE layer.
 * @param cbFct_ pointer to the callback function
 * @return void
 */
EXTERNAL_ void RNET_SetRTERxMsgCbFct(const RAPP_RxMsg_CbFct *cbFct_);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !RNET_TYPES_H_ */
