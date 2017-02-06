/**
 * \file
 * \brief This is main application file
 * \author (c) 2016 Erich Styger, http://mcuoneclipse.com/
 * \note MIT License (http://opensource.org/licenses/mit-license.html)
 *
 * This module implements the application part of the program.
 */

#define MASTER_RNET_APPL_C_

#include "Platform.h"
#include "rnet.h"
#include "RNetConf.h"
#include "Radio.h"
#include "RStack.h"
#include "RApp.h"
#include "CLS1.h"
#include "sh.h"

static RNWK_ShortAddrType dstAddr = RNWK_ADDR_BROADCAST; /* destination node address */

typedef enum RNET_State_s{
  RNET_NONE,
  RNET_POWERUP, /* powered up */
  RNET_TX_RX,
} RNET_State_t;

static uint8_t RNET_HdlRTERxMsgCbFct(RAPP_MSG_Type type_, uint8_t size_, uint8_t *data_, RAPP_ShortAddrType srcAddr_, bool *handled_, RPHY_PacketDesc *pkt_);
static uint8_t RNET_HandleDataRxMessage(RAPP_MSG_Type type, uint8_t size, uint8_t *data, RAPP_ShortAddrType srcAddr, bool *handled, RPHY_PacketDesc *packet);
static void RNET_RadioPowerUp(void);
static uint8_t RNET_PrintStatus(const CLS1_StdIOType *io);
static void RNET_PrintHelp(const CLS1_StdIOType *io);


static RNET_State_t rnetState = RNET_NONE;
static const RAPP_MsgHandler handlerTable[] =
{
  RNET_HandleDataRxMessage,
  RNET_HdlRTERxMsgCbFct,
  NULL /* sentinel */
};

static uint8_t RNET_HdlRTERxMsgCbFct(RAPP_MSG_Type type_, uint8_t size_, uint8_t *data_, RAPP_ShortAddrType srcAddr_, bool *handled_, RPHY_PacketDesc *pktDes_)
{
	StdRtn_t retVal= ERR_PARAM_ADDRESS;
	RTE_RFRxMsgCbFct_t *rxMsgCbFct = NULL;
	RTE_RFPktDes_t pktDes = {0u};
	rxMsgCbFct =RTE_Get_RFRxMsgCbFct();
	if((NULL != rxMsgCbFct) && (NULL != data_) && (NULL != pktDes_))
	{
		pktDes.flags = (uint8)pktDes_->flags;
		pktDes.size  = (uint8)pktDes_->phySize;
		pktDes.data  = (uint8 *)pktDes_->phyData;
		pktDes.rxtx  = (uint8 *)pktDes_->rxtx;
		rxMsgCbFct((RTE_RFMsgType_t)type_, (uint8)size_, (const uint8 *)data_, (uint8)srcAddr_, (uint8 *)handled_, &pktDes);
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
      SH_SendStr((unsigned char*)"Data: ");
      buf[0] = '\0';
      UTIL1_Num8uToStr(buf, sizeof(buf), val);
      SH_SendStr(buf);
      SH_SendStr((unsigned char*)" from addr 0x");
      buf[0] = '\0';
#if RNWK_SHORT_ADDR_SIZE==1
      UTIL1_strcatNum8Hex(buf, sizeof(buf), srcAddr);
#else
      UTIL1_strcatNum16Hex(buf, sizeof(buf), srcAddr);
#endif
      UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
      SH_SendStr(buf);
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

static uint8_t RNET_PrintStatus(const CLS1_StdIOType *io) {
  uint8_t buf[32];
  
  CLS1_SendStatusStr((unsigned char*)"rapp", (unsigned char*)"\r\n", io->stdOut);
  
  UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"0x");
#if RNWK_SHORT_ADDR_SIZE==1
  UTIL1_strcatNum8Hex(buf, sizeof(buf), dstAddr);
#else
  UTIL1_strcatNum16Hex(buf, sizeof(buf), APP_dstAddr);
#endif
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
  CLS1_SendStatusStr((unsigned char*)"  dest addr", buf, io->stdOut);
  
  return ERR_OK;
}

static void RNET_PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"rapp", (unsigned char*)"Group of application commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help", (unsigned char*)"Shows radio help or status\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  saddr 0x<addr>", (unsigned char*)"Set source node address\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  daddr 0x<addr>", (unsigned char*)"Set destination node address\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  send val <val>", (unsigned char*)"Send a value to the destination node\r\n", io->stdOut);
}

void RNET_Init(void) {
  RNET1_Init(); /* initialize stack */
  if (RAPP_SetMessageHandlerTable(handlerTable)!=ERR_OK) { /* assign application message handler */
    SH_SendErrStr((unsigned char*)"ERR: failed setting message handler!\r\n");
  }
  if (RAPP_SetThisNodeAddr(RNWK_ADDR_BROADCAST)!=ERR_OK) { /* set a default address */
     SH_SendErrStr((unsigned char*)"ERR: Failed setting node address\r\n");
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

uint8_t RNET_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  uint8_t res = ERR_OK;
  const uint8_t *p;
  uint16_t val16;
  uint8_t val8;

  if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"rapp help")==0) {
    RNET_PrintHelp(io);
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"rapp status")==0) {
    *handled = TRUE;
    return RNET_PrintStatus(io);
  } else if (UTIL1_strncmp((char*)cmd, (char*)"rapp saddr", sizeof("rapp saddr")-1)==0) {
    p = cmd + sizeof("rapp saddr")-1;
    *handled = TRUE;
    if (UTIL1_ScanHex16uNumber(&p, &val16)==ERR_OK) {
      (void)RNWK_SetThisNodeAddr((RNWK_ShortAddrType)val16);
    } else {
      CLS1_SendStr((unsigned char*)"ERR: wrong address\r\n", io->stdErr);
      return ERR_FAILED;
    }
  } else if (UTIL1_strncmp((char*)cmd, (char*)"rapp send val", sizeof("rapp send val")-1)==0) {
    p = cmd + sizeof("rapp send val")-1;
    *handled = TRUE;
    if (UTIL1_ScanDecimal8uNumber(&p, &val8)==ERR_OK) {
      (void)RAPP_SendPayloadDataBlock(&val8, sizeof(val8), (uint8_t)MSG_TYPE_TESTDATA, dstAddr, RPHY_PACKET_FLAGS_NONE); /* only send low byte */
    } else {
      CLS1_SendStr((unsigned char*)"ERR: wrong number format\r\n", io->stdErr);
      return ERR_FAILED;
    }
  } else if (UTIL1_strncmp((char*)cmd, (char*)"rapp daddr", sizeof("rapp daddr")-1)==0) {
    p = cmd + sizeof("rapp daddr")-1;
    *handled = TRUE;
    if (UTIL1_ScanHex16uNumber(&p, &val16)==ERR_OK) {
      dstAddr = val16;
    } else {
      CLS1_SendStr((unsigned char*)"ERR: wrong address\r\n", io->stdErr);
      return ERR_FAILED;
    }
  }
  return res;
}

RAPP_ShortAddrType RNET_GtDestAddr(void) {
  return dstAddr;
}

void RNET_SetDstAddr(RAPP_ShortAddrType addr_)
{
  dstAddr=addr_;
  return;
}

#ifdef MASTER_RNET_APPL_C_
#undef MASTER_RNET_APPL_C_
#endif


