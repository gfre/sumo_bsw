/***********************************************************************************************//**
 * @file		rnet_clshdlr.c
 * @ingroup		rnet
 * @brief 		Implementation of the command line shell handler for the SWC @a RNet
 *
 * This module implements the interface of the SWC @ref rnet which is addressed to
 * the SWC @ref sh. It introduces application specific commands for requests
 * of status information via command line shell (@b CLS).
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	17.02.2017
 *  
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_rnet_clshdlr_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "rnet_clshdlr.h"
#include "rnet_api.h"
#include "RNWK.h"
#include "UTIL1.h"

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t RNET_PrintStatus(const CLS1_StdIOType *io);
static void RNET_PrintHelp(const CLS1_StdIOType *io);



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t RNET_PrintStatus(const CLS1_StdIOType *io) {
  uint8_t buf[32];

  CLS1_SendStatusStr((unsigned char*)"rapp", (unsigned char*)"\r\n", io->stdOut);

  UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"0x");
#if RNWK_SHORT_ADDR_SIZE==1
  UTIL1_strcatNum8Hex(buf, sizeof(buf), (uint8_t)RNET_GetDstAddr());
#else
  UTIL1_strcatNum16Hex(buf, sizeof(buf), (uint8_t)RNET_GetDstAddr());
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



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
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
      (void)RAPP_SendPayloadDataBlock(&val8, sizeof(val8), (uint8_t)MSG_TYPE_TESTDATA, RNET_GetDstAddr(), RPHY_PACKET_FLAGS_NONE); /* only send low byte */
    } else {
      CLS1_SendStr((unsigned char*)"ERR: wrong number format\r\n", io->stdErr);
      return ERR_FAILED;
    }
  } else if (UTIL1_strncmp((char*)cmd, (char*)"rapp daddr", sizeof("rapp daddr")-1)==0) {
    p = cmd + sizeof("rapp daddr")-1;
    *handled = TRUE;
    if (UTIL1_ScanHex16uNumber(&p, &val16)==ERR_OK) {
    	RNET_SetDstAddr(val16);
    } else {
      CLS1_SendStr((unsigned char*)"ERR: wrong address\r\n", io->stdErr);
      return ERR_FAILED;
    }
  }
  return res;
}



#ifdef MASTER_rnet_clshdlr_C_
#undef MASTER_rnet_clshdlr_C_
#endif /* !MASTER_rnet_clshdlr_C_ */
