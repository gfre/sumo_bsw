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
#include "Acon_Types.h"
#include "sh.h"
#include "sh_cfg.h"
#include "sh_Types.h"
#include "id_Types.h"
#include "RTT1.h"


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



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void SH_PrintWelcomeMsg(const CLS1_StdIOType *io_);
static void SH_PrintGoodByeMsg(const CLS1_StdIOType *io_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static const SH_IOCfg_t *ioCfg = NULL;
static const SH_IODesc_t *ios = NULL;


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static void PrintWelcomeMsg(const CLS1_StdIOType *io_)
{
  uint8 sumoId;
  const char_t verStr[] = {SW_VERSION_CHAR_ARRAY};
  sumoId = Get_SumoID();
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


static void PrintGoodByeMsg(const CLS1_StdIOType *io_)
{
  uint8 sumoId;
  sumoId = Get_SumoID();
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



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void SH_Init(void)
{
	uint8 i = 0u;

	CLS1_Init();
	ioCfg     = Get_ShIOCfg();
	ios       = ioCfg->ios;

	if(( NULL != ioCfg ) && ( NULL != ios ) )
	{
		/* initialize buffers */
		for( i = 0; i < ioCfg->ioSize; i++)
		{
			if( ( NULL != ios[i].buf ) && ( NULL != ios[i].stdio ) )
			{
				/* Eat previous lines */
				while(TRUE == CLS1_ReadLine(ios[i].buf, ios[i].buf, ios[i].bufSize, ios[i].stdio));
				ios[i].buf[0] = '\0';
				PrintWelcomeMsg(ios[i].stdio);
			}
			else
			{
				/*TODO print error msg */
				SH_SENDERRSTR("Error 2\r\n");
			}
		}
	}
	else
	{
		SH_SENDERRSTR("Error 1\r\n");
	}
}


void SH_Deinit(void)
{
  uint8 i = 0u;

  CLS1_Deinit();
  for( i = 0; i < ioCfg->ioSize; i++)
  {
      /* Eat previous lines */
      while(TRUE == CLS1_ReadLine(ios[i].buf, ios[i].buf, ios[i].bufSize, ios[i].stdio));
      ios[i].buf[0] = '\0';
      PrintGoodByeMsg(ios[i].stdio);
  }
  ioCfg = NULL;
  ios = NULL;

}

void SH_MainFct(void)
{
  uint8 i = 0u;
  /* process all I/Os */
  for( i = 0; i < ioCfg->ioSize; i++)
  {
      (void)CLS1_ReadAndParseWithCommandTable(ios[i].buf, ios[i].bufSize, ios[i].stdio, Get_CmdParserTbl());
  }
}

#ifdef MASTER_SH_C_
#undef MASTER_SH_C_
#endif

