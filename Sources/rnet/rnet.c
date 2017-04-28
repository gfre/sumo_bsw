/***********************************************************************************************//**
 * @file		rnet.c
 * @ingroup		rnet
 * @brief 		This is main application file
 *
 * This software component implements an application entry layer for the Radio Network Stack. It
 * runs a state machine where the radio is powered up and the radio network stack gets processed.
 * Furthermore the it implements callback functions for handling received message and allows to
 * set and get the destination addresses. The component includes the firmware component @a RApp.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
  * @date 	06.01.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_RNET_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "rnet.h"
#include "rnet_api.h"
#include "sh_Types.h"
#include "RNetConf.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum RNET_State_s{
  RNET_NONE,
  RNET_POWERUP, /* powered up */
  RNET_TX_RX,
} RNET_State_t;



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t RNET_HdlRTERxMsgCbFct(RAPP_MSG_Type type_, uint8_t size_, uint8_t *data_, RAPP_ShortAddrType srcAddr_, bool *handled_, RPHY_PacketDesc *pktDes_);
static uint8_t RNET_HandleDataRxMessage(RAPP_MSG_Type type, uint8_t size, uint8_t *data, RAPP_ShortAddrType srcAddr, bool *handled, RPHY_PacketDesc *packet);
static void RNET_RadioPowerUp(void);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static RNWK_ShortAddrType dstAddr = RNWK_ADDR_BROADCAST; /* destination node address */
static RNET_State_t rnetState = RNET_NONE;
static RAPP_RxMsg_CbFct *rteRxMsgCbFct = NULL;

static const RAPP_MsgHandler handlerTable[] =
{
  RNET_HandleDataRxMessage,
  RNET_HdlRTERxMsgCbFct,
  NULL /* sentinel */
};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t RNET_HdlRTERxMsgCbFct(RAPP_MSG_Type type_, uint8_t size_, uint8_t *data_, RAPP_ShortAddrType srcAddr_, bool *handled_, RPHY_PacketDesc *pktDes_)
{
	StdRtn_t retVal= ERR_PARAM_ADDRESS;
	RAPP_PktDesc pktDes = {0u};
	if((NULL != rteRxMsgCbFct) && (NULL != data_) && (NULL != pktDes_))
	{
		pktDes.flags = (uint8)pktDes_->flags;
		pktDes.size  = (uint8)pktDes_->phySize;
		pktDes.data  = (uint8 *)pktDes_->phyData;
		pktDes.rxtx  = (uint8 *)pktDes_->rxtx;
		rteRxMsgCbFct(type_, (uint8)size_, (const uint8 *)data_, (uint8)srcAddr_, (uint8 *)handled_, &pktDes);
		retVal = ERR_OK;
	}
	return retVal;

}
static uint8_t RNET_HandleDataRxMessage(RAPP_MSG_Type type, uint8_t size, uint8_t *data, RAPP_ShortAddrType srcAddr, bool *handled, RPHY_PacketDesc *packet) {
  uint8_t buf[32];
  uint8_t val;
  
  (void)size;
  (void)packet;
  switch(type) {
    case MSG_TYPE_TESTDATA: /* generic data message */
      *handled = TRUE;
      val = *data; /* get data value */
      SH_SENDSTR((unsigned char*)"Data: ");
      buf[0] = '\0';
      UTIL1_Num8uToStr(buf, sizeof(buf), val);
      SH_SENDSTR(buf);
      SH_SENDSTR((unsigned char*)" from addr 0x");
      buf[0] = '\0';
#if RNWK_SHORT_ADDR_SIZE==1
      UTIL1_strcatNum8Hex(buf, sizeof(buf), srcAddr);
#else
      UTIL1_strcatNum16Hex(buf, sizeof(buf), srcAddr);
#endif
      UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
      SH_SENDSTR(buf);
      return ERR_OK;
    default:
      break;
  } /* switch */
  return ERR_OK;
}



static void RNET_RadioPowerUp(void) {
  /* need to ensure that we wait 100 ms after power-on of the transceiver */
  portTickType xTime;
  
  xTime = FRTOS1_xTaskGetTickCount();
  if (xTime<(100/portTICK_PERIOD_MS)) {
    /* not powered for 100 ms: wait until we can access the radio transceiver */
    xTime = (100/portTICK_PERIOD_MS)-xTime; /* remaining ticks to wait */
    FRTOS1_vTaskDelay(xTime);
  }
  (void)RNET1_PowerUp();
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void RNET_Init(void) {
  RNET1_Init(); /* initialize stack */
  if (RAPP_SetMessageHandlerTable(handlerTable)!=ERR_OK)
  {
	  /* assign application message handler */
	  SH_SENDERRSTR((unsigned char*)"ERR: failed setting message handler!\r\n");
  }
  if (RAPP_SetThisNodeAddr(RNWK_ADDR_BROADCAST)!=ERR_OK)
  {
	  /* set a default address */
	  SH_SENDERRSTR((unsigned char*)"ERR: Failed setting node address\r\n");
  }
  rnetState = RNET_NONE;
}


void RNET_MainFct(void) {
  for(;;) {
    switch(rnetState) {
    case RNET_NONE:
      rnetState = RNET_POWERUP;
      continue;

    case RNET_POWERUP:
      RNET_RadioPowerUp();
      rnetState = RNET_TX_RX;
      break;

    case RNET_TX_RX:
      (void)RNET1_Process();
      break;

    default:
      break;
    } /* switch */
    break; /* break for loop */
  } /* for */
}



RAPP_ShortAddrType RNET_GetDstAddr(void)
{
  return dstAddr;
}

void RNET_SetDstAddr(RAPP_ShortAddrType addr_)
{
  dstAddr = addr_;
  return;
}

RAPP_RxMsg_CbFct *RNET_GetRTERxMsgCbFct(void)
{
  return rteRxMsgCbFct;
}

void RNET_SetRTERxMsgCbFct(const RAPP_RxMsg_CbFct *cbFct_)
{
	rteRxMsgCbFct = cbFct_;
  return;
}


#ifdef MASTER_RNET_C_
#undef MASTER_RNET_C_
#endif


