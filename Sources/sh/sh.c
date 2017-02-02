/*******************************************************************************
 * @brief 	Shell and console interface implementation.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 	02.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements the front to the console/shell functionality.
 *
 * ==============================================================================
 */

#define MASTER_SH_C_

#include "sh.h"
#include "CLS1.h"
#include "FRTOS1.h"
#include "appl.h"
#include "Buzzer.h"
#include "Motor.h"
#include "RNET1.h"
#include "LED1.h"
#include "LED2.h"
#include "batt.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "Tacho.h"
#include "KIN1.h"
#include "Pid.h"
#include "drv.h"
#include "RTT1.h"
#include "id.h"
#include "nvm.h"
#include "state.h"
#include "rnet.h"

void SH_SendString(unsigned char *msg) {
  CLS1_SendStr(msg, SH_GetStdio()->stdOut);
  CLS1_SendStr(msg, RTT1_stdio.stdOut);

}

static uint8_t SH_PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"shell", (unsigned char*)"Group of shell commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows shell help or status\r\n", io->stdOut);
  return ERR_OK;
}

static uint8_t SH_PrintStatus(const CLS1_StdIOType *io) {
  CLS1_SendStatusStr((unsigned char*)"shell", (unsigned char*)"\r\n", io->stdOut);
  CLS1_SendStatusStr((unsigned char*)"  connections", NULL, io->stdOut);
  CLS1_SendStr((unsigned char*)"DEFAULT", io->stdOut);
  CLS1_SendStr((unsigned char*)"   +RTT", io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  return ERR_OK;
}

static uint8_t SH_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  uint8_t res = ERR_OK;

  if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"shell help")==0) {
    *handled = TRUE;
    return SH_PrintHelp(io);
  } else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"shell status")==0) {
    *handled = TRUE;
    return SH_PrintStatus(io);
  }
  return res;
}

static const CLS1_ParseCommandCallback CmdParserTable[] =
{
  CLS1_ParseCommand,
  SH_ParseCommand,
  FRTOS1_ParseCommand,
  APPL_ParseCommand,
  STATE_ParseCommand,
  NVM_ParseCommand,
  MOT_ParseCommand,
  DRV_ParseCommand,
  TACHO_ParseCommand,
  PID_ParseCommand,
  Q4CLeft_ParseCommand,
  Q4CRight_ParseCommand,
  BUZ_ParseCommand,
  LED1_ParseCommand,
  LED2_ParseCommand,
  RNET1_ParseCommand,
  BATT_ParseCommand,
  KIN1_ParseCommand,
  RNET_ParseCommand,
  NULL /* Sentinel */
};

typedef struct SH_IODesc_s{
  unsigned char *buf;
  size_t bufSize;
  CLS1_ConstStdIOType *stdio;
} SH_IODesc;


  CLS1_ConstStdIOType *SH_GetStdio(void) {
    return CLS1_GetStdio();
  }


static const SH_IODesc ios[] =
{
    {CLS1_DefaultShellBuffer, sizeof(CLS1_DefaultShellBuffer), &CLS1_stdio},
    {RTT1_DefaultShellBuffer, sizeof(RTT1_DefaultShellBuffer), &RTT1_stdio},
};


void SH_MainFct(void)
{
	uint8 i = 0u;
	/* process all I/Os */
    for(i=0;i<sizeof(ios)/sizeof(ios[0]);i++)
    {
      (void)CLS1_ReadAndParseWithCommandTable(ios[i].buf, ios[i].bufSize, ios[i].stdio, CmdParserTable);
    }
}


void SH_Init(void)
{
     uint8 i = 0u;
	 uint8 buf[32];
	 uint8 sumoId;

	/* initialize buffers */
	for(i=0;i<sizeof(ios)/sizeof(ios[0]);i++) {
		ios[i].buf[0] = '\0';
	}

	SH_SendString("Shell task started!\r\n");

	/* print ID information about current sumo to the shell welcome dialog*/
	sumoId = ID_WhichSumo();
	if (sumoId == ERR_PARAM_ADDRESS)
	{
		UTIL1_strcpy(buf, sizeof(buf), "Sorry! Your sumo is unknown.");
    }
	else
	{
		UTIL1_strcpy(buf, sizeof(buf), "Welcome to Sumo #");
	  	UTIL1_strcatNum8u(buf, sizeof(buf), ID_WhichSumo());
	  	UTIL1_strcat(buf, sizeof(buf), " \r\n");
	}
	SH_SendString(buf);

}

void SH_ParseCmd(unsigned char *cmd) {
  (void)CLS1_ParseWithCommandTable(cmd, SH_GetStdio(), CmdParserTable);
}

void SH_Deinit(void) {
  /* nothing to do */
  CLS1_Deinit();
}

#ifdef MASTER_SH_C_
#undef MASTER_SH_C_
#endif

