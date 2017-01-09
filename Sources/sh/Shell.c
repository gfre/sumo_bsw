/**
 * \file
 * \brief Shell and console interface implementation.
 * \author Erich Styger
 *
 * This module implements the front to the console/shell functionality.
 */

#include "Shell.h"
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


void SHELL_SendString(unsigned char *msg) {
  CLS1_SendStr(msg, SHELL_GetStdio()->stdOut);

}

static uint8_t SHELL_PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"shell", (unsigned char*)"Group of shell commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows shell help or status\r\n", io->stdOut);
  return ERR_OK;
}

static uint8_t SHELL_PrintStatus(const CLS1_StdIOType *io) {
  CLS1_SendStatusStr((unsigned char*)"shell", (unsigned char*)"\r\n", io->stdOut);
  CLS1_SendStatusStr((unsigned char*)"  connections", NULL, io->stdOut);
  CLS1_SendStr((unsigned char*)"DEFAULT", io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
  return ERR_OK;
}

static uint8_t SHELL_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  uint8_t res = ERR_OK;

  if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"shell help")==0) {
    *handled = TRUE;
    return SHELL_PrintHelp(io);
  } else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"shell status")==0) {
    *handled = TRUE;
    return SHELL_PrintStatus(io);
  }
  return res;
}

static const CLS1_ParseCommandCallback CmdParserTable[] =
{
  CLS1_ParseCommand,
  SHELL_ParseCommand,
  FRTOS1_ParseCommand,
  APP_ParseCommand,
  MOT_ParseCommand,
  BUZ_ParseCommand,
  LED1_ParseCommand,
  LED2_ParseCommand,
  RNET1_ParseCommand,
  Q4CLeft_ParseCommand,
  Q4CRight_ParseCommand,
  BATT_ParseCommand,
  NULL /* Sentinel */
};

typedef struct {
  unsigned char *buf;
  size_t bufSize;
  CLS1_ConstStdIOType *stdio;
} SHELL_IODesc;


  CLS1_ConstStdIOType *SHELL_GetStdio(void) {
    return CLS1_GetStdio();
  }


static const SHELL_IODesc ios[] =
{
    {CLS1_DefaultShellBuffer, sizeof(CLS1_DefaultShellBuffer), &CLS1_stdio}
};


static void ShellTask (void *pvParameters) {
  unsigned int i;

  (void)pvParameters; /* not used */
  /* initialize buffers */
  for(i=0;i<sizeof(ios)/sizeof(ios[0]);i++) {
    ios[i].buf[0] = '\0';
  }
  SHELL_SendString("Shell task started!\r\n");

  for(;;) {
	/* process all I/Os */
    for(i=0;i<sizeof(ios)/sizeof(ios[0]);i++) {
      (void)CLS1_ReadAndParseWithCommandTable(ios[i].buf, ios[i].bufSize, ios[i].stdio, CmdParserTable);
    }

    FRTOS1_vTaskDelay(10/portTICK_PERIOD_MS);
  } /* for */
}


void SHELL_Init(void) {

  if (FRTOS1_xTaskCreate(ShellTask, "Shell", configMINIMAL_STACK_SIZE+50, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error */
  }

}

void SHELL_ParseCmd(unsigned char *cmd) {
  (void)CLS1_ParseWithCommandTable(cmd, SHELL_GetStdio(), CmdParserTable);
}

void SHELL_Deinit(void) {
  /* nothing to do */
  CLS1_Deinit();
}

