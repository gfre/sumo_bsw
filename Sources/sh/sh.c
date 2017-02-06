/***************************************************************************************************
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
 * =================================================================================================
 */

#define MASTER_SH_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"
#include "sh.h"
#include "sh_cfg.h"
#include "appl_cfg.h"
#include "CLS1.h"
#include "RTT1.h"
#include "id.h"



/*======================================= >> #DEFINES << =========================================*/
#define SH_DASH_LINE         (CLS1_DASH_LINE)
#define SH_STAR_LINE         ("\r\n**************************************************************\r\n")
#define SH_DOUBLE_DASH_LINE  (    "==============================================================\r\n")
#define SH_WELCOME_LINE1     (    "        >> Welcome to the ACon Sumo Robots Project <<         \r\n")
#define SH_WELCOME_LINE2     (    "         -------------------------------------------          \r\n")
#define SH_SUMO_INTRO_LINE1  ("\r\n  Hello, my name is SULLEN SUMO and my ID is #%d. I am highly\r\n")
#define SH_SUMO_INTRO_LINE2  (    "  trained with basic software version %s%s.\r\n")
#define SH_SHELL_INTRO_LINE1 ("\r\n  This is a command line shell for debugging the SUMO basic \r\n")
#define SH_SHELL_INTRO_LINE2 (    "  software and firmware. Type \"help\" for usage documentation.\r\n")
#define SH_COPYRIGHT_LINE1   ("\r\n  (c)2017 Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel\r\n")
#define SH_COPYRIGHT_LINE2   (    "          Erich Styger, erich.styger@hslu.ch, HSLU Luzern\r\n")
#define SH_GOODBYE_LINE1     (    "           ***  Good Bye from SULLEN SUMO #%d ***              \r\n")
#define SH_GOODBYE_LINE2     ("\r\n    Thank you for debugging the ACon Sumo Robots Project!      \r\n")
#define SH_THANKS_LINE1      ("\r\n  Special thanks go to Prof. Erich Styger from the HSLU Luzern.\r\n")
#define SH_THANKS_LINE2      (    "  >> Visit him on www.MCUonEclipse.com! \r\n")

/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct SH_IODesc_s{
  unsigned char *buf;
  size_t bufSize;
  CLS1_ConstStdIOType *stdio;
} SH_IODesc;


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t SH_PrintHelp(const CLS1_StdIOType *io);
static uint8_t SH_PrintStatus(const CLS1_StdIOType *io);
static void SH_PrintWelcomeMsg(const CLS1_StdIOType *io_);
static void SH_ExitShTask(void);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static const SH_IODesc ios[] =
{
    {CLS1_DefaultShellBuffer, sizeof(CLS1_DefaultShellBuffer), &CLS1_stdio},
    {RTT1_DefaultShellBuffer, sizeof(RTT1_DefaultShellBuffer), &RTT1_stdio},
};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
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


void SH_SendString(unsigned char *msg) {
  CLS1_SendStr(msg, CLS1_GetStdio()->stdOut);
  CLS1_SendStr(msg, RTT1_stdio.stdOut);
}



static void SH_PrintWelcomeMsg(const CLS1_StdIOType *io_)
{
  uint8 sumoId;
  const char_t verStr[] = {SW_VERSION_CHAR_ARRAY};
  sumoId = ID_WhichSumo();
  CLS1_SendStr((const uint8 *)SH_STAR_LINE, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_WELCOME_LINE1, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_WELCOME_LINE2, io_->stdOut);
  CLS1_printfIO(io_, (const char_t *)SH_SUMO_INTRO_LINE1, (int8)sumoId);
  CLS1_printfIO(io_, (const char_t *)SH_SUMO_INTRO_LINE2, verStr, (const char_t *)SW_YEAR);
  CLS1_SendStr((const uint8 *)SH_SHELL_INTRO_LINE1, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_SHELL_INTRO_LINE2, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_COPYRIGHT_LINE1, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_COPYRIGHT_LINE2, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_DOUBLE_DASH_LINE, io_->stdOut);
  CLS1_SendStr((const uint8 *)"\r\n", io_->stdOut);
  CLS1_PrintPrompt(io_);

  return;
}


static void SH_PrintGoodByeMsg(const CLS1_StdIOType *io_)
{
  uint8 sumoId;
  sumoId = ID_WhichSumo();
  CLS1_SendStr((const uint8 *)"\r\n\r\n", io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_DASH_LINE, io_->stdOut);
  CLS1_SendStr((const uint8 *)"\r\n\r\n", io_->stdOut);
  CLS1_printfIO(io_, (const char_t *)SH_GOODBYE_LINE1, (int8)sumoId);
  CLS1_SendStr((const char_t *)SH_GOODBYE_LINE2, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_THANKS_LINE1, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_THANKS_LINE2, io_->stdOut);
  CLS1_SendStr((const uint8 *)SH_DASH_LINE, io_->stdOut);

  return;
}

static void SH_ExitShTask(void)
{
  const APPL_TaskCfgItm_t *mainTaskCfg = NULL;
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  mainTaskCfg = Get_APPL_MainTaskCfg();
  if ((NULL != mainTaskCfg) && (mainTaskCfg->taskHdl))
  {
      FRTOS1_xTaskNotifyFromISR( mainTaskCfg->taskHdl,
				 KEY_PRESSED_LONG_NOTIFICATION_VALUE,
				 eSetBits,
				 &higherPriorityTaskWoken );
      portYIELD_FROM_ISR( higherPriorityTaskWoken );
  }
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void SH_SendStr(unsigned char *msg_)
{
  CLS1_SendStr(msg_, CLS1_GetStdio()->stdOut);
  CLS1_SendStr(msg_, RTT1_GetStdio()->stdOut);
}

void SH_SendErrStr(unsigned char *msg_)
{
  CLS1_SendStr(msg_, CLS1_GetStdio()->stdErr);
  CLS1_SendStr(msg_, RTT1_GetStdio()->stdErr);
}

uint8_t SH_ParseCommand(const unsigned char *cmd_, bool *handled_, const CLS1_StdIOType *io_)
{
  uint8_t res = ERR_OK;

  *handled_ = FALSE;
  if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==ERR_OK || UTIL1_strcmp((char*)cmd_, (char*)"shell help")==ERR_OK) {
      *handled_ = TRUE;
      return SH_PrintHelp(io_);
  } else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==ERR_OK || UTIL1_strcmp((char*)cmd_, (char*)"shell status")==ERR_OK) {
      *handled_ = TRUE;
      return SH_PrintStatus(io_);
  } else if (ERR_OK == UTIL1_strcmp((char*)cmd_, (char*)SH_CMD_EXIT)) {
      SH_ExitShTask();
      *handled_ = TRUE;
  }
  return res;
}


void SH_Init(void)
{
  uint8 i = 0u;

  CLS1_Init();

  /* initialize buffers */
  for(i=0;i<sizeof(ios)/sizeof(ios[0]);i++)
  {
      /* Eat previous lines */
      while(TRUE == CLS1_ReadLine(ios[i].buf, ios[i].buf, ios[i].bufSize, ios[i].stdio));
      ios[i].buf[0] = '\0';
      SH_PrintWelcomeMsg(ios[i].stdio);
  }

}


void SH_Deinit(void)
{
  uint8 i = 0u;

  CLS1_Deinit();
  for(i=0;i<sizeof(ios)/sizeof(ios[0]);i++)
  {
      /* Eat previous lines */
      while(TRUE == CLS1_ReadLine(ios[i].buf, ios[i].buf, ios[i].bufSize, ios[i].stdio));
      ios[i].buf[0] = '\0';
      SH_PrintGoodByeMsg(ios[i].stdio);
  }

}

void SH_MainFct(void)
{
  uint8 i = 0u;
  /* process all I/Os */
  for(i=0;i<sizeof(ios)/sizeof(ios[0]);i++)
  {
      (void)CLS1_ReadAndParseWithCommandTable(ios[i].buf, ios[i].bufSize, ios[i].stdio, Get_CmdParserTbl());
  }
}

#ifdef MASTER_SH_C_
#undef MASTER_SH_C_
#endif

