/***********************************************************************************************//**
 * @file		RNet_AppConfig.h
 * @ingroup		rnet
 * @brief 		API of the SWC @a RNet
 *
 * This header file provides a user configuration file of the SWC @ref rnet. It maps data types from
 * the @ref rte to the application interface of the Radio Network Stack.
 *
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.01.2017
 *
 * @note Data Types for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef RNET_APPCONFIG_
#define RNET_APPCONFIG_

/*======================================= >> #INCLUDES << ========================================*/
#include "rte_Types.h"



#ifdef MASTER_RAPP_C_
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
/**
 *	@brief Data type mapping of the radio message types RTE_RF_MSG_TYPE_T defined on
 *	the @ref rte.
 */
typedef RTE_RF_MSG_TYPE_T		RAPP_MSG_Type;

/**
 * @brief Data type mapping of the package descriptor type RFPktDes_t defined on
 * the @ref rte.
 */
typedef RFPktDes_t      		RAPP_PktDesc;

/**
 * @brief Data type mapping of the function type for the radio receive message callback function
 * RFRxMsgCbFct_t defined on the @ref rte.
 */
typedef RFRxMsgCbFct_t  		RAPP_RxMsg_CbFct;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* __RNET_APPCONFIG__ */
