/*******************************************************************************
 * @brief 	Main state machine of application software layer.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date 	13.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements an (extended) state machine for the Sumo robot. It
 * handles the main states of the application software layer.
 *
 * ==============================================================================
 */

#define MASTER_STATE_C_

#include "Platform.h"
#include "CLS1.h"
#include "RNET1.h"
#include "state.h"
#include "stud.h"
#include "task_cfg.h"
#include "rte.h"
#include "LED1.h"
#include "sh.h"


static uint8_t STATE_PrintHelp(const CLS1_StdIOType *io_);
static StdRtn_t STATE_PrintStatus(const CLS1_StdIOType *io_);
static const uint8 * STATE_ReadStateString(mainState_t state_);
static void STATE_RunStateMachine(void);
static StdRtn_t STATE_msSTARTUP(void);
static StdRtn_t STATE_msINIT(void);
static StdRtn_t STATE_msIDLE(void);
static StdRtn_t STATE_msNORMAL(void);
static StdRtn_t STATE_msDEBUG(void);
static void STATE_msERROR(void);

static mainState_t mainState = MAIN_STATE_STARTUP;

const TASK_CfgItm_t *shTaskCfg = NULL;


static StdRtn_t STATE_SyncStateMachineWithISR()
{
  BaseType_t notfRes = pdFAIL;
  uint32 notfVal = 0u;

  notfRes = FRTOS1_xTaskNotifyWait( pdFALSE,
				    UINT32_MAX,
				    (uint32_t *)&notfVal,
				    pdMS_TO_TICKS( 0u ) );

  /* Transitions from IDLE state */
  if( ( pdPASS == notfRes ) && ( MAIN_STATE_IDLE == mainState ) )
  {
      /* Handle transition from IDLE --> NORMAL */
      if( (notfVal & KEY_RELEASED_NOTIFICATION_VALUE) != FALSE)
      {
	  mainState = MAIN_STATE_NORMAL;
	  RTE_Write_BuzPlayTune(BUZ_TUNE_BUTTON);
      }
      /* Handle transition from IDLE --> DEBUG */
      if( (notfVal & KEY_PRESSED_LONG_NOTIFICATION_VALUE) != FALSE)
      {
	  mainState = MAIN_STATE_DEBUG;
	  SH_Init();
	  FRTOS1_vTaskResume(shTaskCfg->taskHdl);
	  RTE_Write_BuzPlayTune(BUZ_TUNE_ACCEPT);
      }
  }
  /* Transitions from DEBUG state */
  else if( ( pdPASS == notfRes ) && ( MAIN_STATE_DEBUG == mainState ) )
  {
      /* Handle transition from DEBUG --> IDLE */
      if( (notfVal & KEY_PRESSED_LONG_NOTIFICATION_VALUE) != FALSE)
      {
	  mainState = MAIN_STATE_IDLE;
	  SH_Deinit();
	  FRTOS1_vTaskSuspend(shTaskCfg->taskHdl);
	  RTE_Write_BuzPlayTune(BUZ_TUNE_DECLINE);
      }
  }
  else
  {

  }
}

static void STATE_RunStateMachine(void) {
  static StdRtn_t retVal = ERR_OK;
  switch (mainState)
  {
    case MAIN_STATE_STARTUP:
    {
      retVal = STATE_msSTARTUP();
      if(ERR_OK == retVal)
      {
	      mainState = MAIN_STATE_INIT;
      }
      else
      {
	      mainState = MAIN_STATE_ERROR;
      }
      break;
    }
    case MAIN_STATE_INIT:
    {
      retVal = STATE_msINIT();
      if(ERR_OK == retVal)
      {
	      mainState = MAIN_STATE_IDLE;
      }
      else
      {
	      mainState = MAIN_STATE_ERROR;
      }
      break;
    }
    case MAIN_STATE_IDLE:
    {
      retVal = STATE_msIDLE();
      if(ERR_OK != retVal)
      {
	      mainState = MAIN_STATE_ERROR;
      }
      break;
    }
    case MAIN_STATE_NORMAL:
    {
      retVal = STATE_msNORMAL();
      if(ERR_OK != retVal)
      {
	      mainState = MAIN_STATE_ERROR;
      }
      break;
    }
    case MAIN_STATE_DEBUG:
    {
      retVal = STATE_msDEBUG();
      if(ERR_OK != retVal)
      {
	      mainState = MAIN_STATE_ERROR;
      }
      break;
    }
    default:
    case MAIN_STATE_ERROR:
    {
      STATE_msERROR();
      break;
    }
  }/* switch */

  STATE_SyncStateMachineWithISR();

  return;
}

static StdRtn_t STATE_msSTARTUP(void)
{
	return ERR_OK;
}

static StdRtn_t STATE_msINIT(void)
{
  StdRtn_t retVal = ERR_PARAM_ADDRESS;

  STUD_Init();

  shTaskCfg = TASK_Get_ShTaskCfg();
  if(NULL != shTaskCfg)
  {
	  retVal = ERR_OK;
  }
  return retVal;
}

static StdRtn_t STATE_msIDLE(void)
{
  return ERR_OK;
}

static StdRtn_t STATE_msNORMAL(void)
{
  STUD_Main();
  return ERR_OK;
}

static StdRtn_t STATE_msDEBUG(void)
{
  return ERR_OK;
}
static void STATE_msERROR(void)
{
	return;
}



static uint8_t STATE_PrintHelp(const CLS1_StdIOType *io_) {
  CLS1_SendHelpStr((unsigned char*)"state", (unsigned char*)"Group of state commands\r\n", io_->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows state help or status\r\n", io_->stdOut);
  return ERR_OK;
}

static StdRtn_t STATE_PrintStatus(const CLS1_StdIOType *io_) {
  uint8 retVal = ERR_OK;
  CLS1_SendStatusStr((unsigned char*)"state", (unsigned char*)"\r\n", io_->stdOut);
  CLS1_SendStatusStr((unsigned char*)"  current", STATE_ReadStateString(mainState) , io_->stdOut);
  return retVal;
}

static const uint8 * STATE_ReadStateString(mainState_t state_){
  switch(state_) {
    case MAIN_STATE_STARTUP: return "STARTUP\r\n";
    case MAIN_STATE_INIT:    return "INIT\r\n";
    case MAIN_STATE_IDLE:    return "IDLE\r\n";
    case MAIN_STATE_NORMAL:  return "NORMAL\r\n";
    case MAIN_STATE_DEBUG:  return "DEBUG\r\n";
    case MAIN_STATE_ERROR:   return "ERROR\r\n";
    default: return ">> fatal error - unknown or undefined main state <<\r\n";
  }
}

void STATE_mainFct(void){
  STATE_RunStateMachine();
}


uint8 STATE_ParseCommand(const unsigned char *cmd_, bool *handled, const CLS1_StdIOType *io_) {
  uint8 res = ERR_OK;
  if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd_, (char*)"state help")==0) {
    *handled = TRUE;
    return STATE_PrintHelp(io_);
  } else if (UTIL1_strcmp((char*)cmd_, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd_, (char*)"state status")==0) {
    *handled = TRUE;
    return STATE_PrintStatus(io_);
  }
  return res;
}


#ifdef MASTER_STATE_C_
#undef MASTER_STATE_C_
#endif
