/***********************************************************************************************//**
 * @file		rnet_api.h
 * @ingroup		rnet
 * @brief 		API of the SWC *Radio Net*
 *
 * This API provides a BSW-internal interface of the SWC @ref rnet. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	17.02.2017
 *  
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef RNET_API_H_
#define RNET_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "RApp.h"


#ifdef MASTER_RNET_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup rnet
 * @{
 */
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


/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !RNET_API_H_ */
