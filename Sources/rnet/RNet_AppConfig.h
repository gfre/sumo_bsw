/*******************************************************************************
 * @brief 	This is a configuration file for the RNet stack.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 	06.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#ifndef __RNET_APPCONFIG__
#define __RNET_APPCONFIG__


/*! type ID's for application messages */
typedef enum {
  RAPP_MSG_TYPE_DATA = 0x04,                    /* generic data message */
  RAPP_MSG_TYPE_PING = 0x55,
  RAPP_MSG_TYPE_BUTTON = 0x56,
} RAPP_MSG_Type;

#endif /* __RNET_APPCONFIG__ */


#define RNET_CONFIG_TRANSCEIVER_PAYLOAD_SIZE  32
  /*!< Size of the physical transceiver payload (bytes), max 32 bytes for nRF24L01+, max 128 bytes for MC1320x */

#define RNET_CONFIG_SHORT_ADDR_SIZE   1
  /*!< size of short address type. Either 1 or 2 */
