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
 * @brief Return the current network address of the RF-node.
 * @return network address
 */
EXTERNAL_ RAPP_ShortAddrType RNET_GetDstAddr(void);

/**
 * @brief Set the network address of the RF-node.
 * @param addr_ network address
 * @return void
 */
EXTERNAL_ void RNET_SetDstAddr(RAPP_ShortAddrType addr_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !RNET_TYPES_H_ */
