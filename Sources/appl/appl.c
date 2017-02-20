/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file appl.c
 * 
 *==================================================================================================
 */

#define MASTER_appl_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "appl.h"
#include "appl_Types.h"
#include "Platform.h"
#include "RNET1.h"
#include "stud.h"
#include "task_cfg.h"
#include "rte.h"
#include "LED1.h"
#include "sh.h"

#ifdef ASW_ENABLED
#include "asw.h"
#endif

/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/


/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static void APPL_RunStateMachine(void);
static StdRtn_t APPL_msSTARTUP(void);
static StdRtn_t APPL_msINIT(void);
static StdRtn_t APPL_msIDLE(void);
static StdRtn_t APPL_msNORMAL(void);
static StdRtn_t APPL_msDEBUG(void);
static void APPL_msERROR(void);
static inline StdRtn_t Set_State(const APPL_State_t state_);


/*=================================== >> GLOBAL VARIABLES << =====================================*/
APPL_State_t applState = APPL_STATE_NONE;
const TASK_CfgItm_t *dbgTaskCfg = NULL;

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static inline StdRtn_t Set_State(const APPL_State_t state_)
{
	StdRtn_t retVal = ERR_PARAM_RANGE;
	if( ( APPL_STATE_NONE < state_ ) && (APPL_STATE_NUM > state_ ) )
	{
		applState = state_;
		retVal = ERR_OK;
	}
	return retVal;
}

static StdRtn_t APPL_SyncStateMachineWithISR()
{
  BaseType_t notfRes = pdFAIL;
  uint32 notfVal = 0u;

  notfRes = FRTOS1_xTaskNotifyWait( pdFALSE,
				    UINT32_MAX,
				    (uint32_t *)&notfVal,
				    pdMS_TO_TICKS( 0u ) );

  /* Transitions from IDLE state */
  if( ( pdPASS == notfRes ) && ( APPL_STATE_IDLE == applState ) )
  {
      /* Handle transition from IDLE --> NORMAL */
      if( (notfVal & KEY_RELEASED_NOTIFICATION_VALUE) != FALSE)
      {
    	  Set_State(APPL_STATE_NORMAL);
    	  RTE_Write_BuzPlayTune(BUZ_TUNE_BUTTON);
      }
      /* Handle transition from IDLE --> DEBUG */
      if( (notfVal & KEY_PRESSED_LONG_NOTIFICATION_VALUE) != FALSE)
      {
    	  Set_State(APPL_STATE_DEBUG);
    	  SH_Init();
    	  FRTOS1_vTaskResume(dbgTaskCfg->taskHdl);
    	  RTE_Write_BuzPlayTune(BUZ_TUNE_ACCEPT);
      }
  }
  /* Transitions from DEBUG state */
  else if( ( pdPASS == notfRes ) && ( APPL_STATE_DEBUG == applState ) )
  {
      /* Handle transition from DEBUG --> IDLE */
      if( (notfVal & KEY_PRESSED_LONG_NOTIFICATION_VALUE) != FALSE)
      {
    	  Set_State(APPL_STATE_IDLE);
		  SH_Deinit();
		  FRTOS1_vTaskSuspend(dbgTaskCfg->taskHdl);
		  RTE_Write_BuzPlayTune(BUZ_TUNE_DECLINE);
      }
  }
  else
  {

  }
}

static void APPL_RunStateMachine(void) {
  static StdRtn_t retVal = ERR_OK;
  switch (applState)
  {
    case APPL_STATE_STARTUP:
    {
      retVal = APPL_msSTARTUP();
      if(ERR_OK == retVal)
      {
    	  Set_State(APPL_STATE_INIT);
      }
      else
      {
    	  Set_State(APPL_STATE_ERROR);
      }
      break;
    }
    case APPL_STATE_INIT:
    {
      retVal = APPL_msINIT();
      if(ERR_OK == retVal)
      {
    	  Set_State(APPL_STATE_IDLE);
      }
      else
      {
    	  Set_State(APPL_STATE_ERROR);
      }
      break;
    }
    case APPL_STATE_IDLE:
    {
      retVal = APPL_msIDLE();
      if(ERR_OK != retVal)
      {
    	  Set_State(APPL_STATE_ERROR);
      }
      break;
    }
    case APPL_STATE_NORMAL:
    {
      retVal = APPL_msNORMAL();
      if(ERR_OK != retVal)
      {
    	  Set_State(APPL_STATE_ERROR);
      }
      break;
    }
    case APPL_STATE_DEBUG:
    {
      retVal = APPL_msDEBUG();
      if(ERR_OK != retVal)
      {
    	  Set_State(APPL_STATE_ERROR);
      }
      break;
    }
    default:
    case APPL_STATE_ERROR:
    {
      APPL_msERROR();
      break;
    }
  }/* switch */

  APPL_SyncStateMachineWithISR();

  return;
}

static StdRtn_t APPL_msSTARTUP(void)
{
	return ERR_OK;
}

static StdRtn_t APPL_msINIT(void)
{
  StdRtn_t retVal = ERR_PARAM_ADDRESS;
#ifdef ASW_ENABLED
  ASW_Init();
#endif
  STUD_Init();

  dbgTaskCfg = TASK_Get_DbgTaskCfg();
  if(NULL != dbgTaskCfg)
  {
	  retVal = ERR_OK;
  }
  return retVal;
}

static StdRtn_t APPL_msIDLE(void)
{
  return ERR_OK;
}

static StdRtn_t APPL_msNORMAL(void)
{
#ifdef ASW_ENABLED
	ASW_Main();
#endif
  STUD_Main();
  return ERR_OK;
}

static StdRtn_t APPL_msDEBUG(void)
{
  return ERR_OK;
}
static void APPL_msERROR(void)
{
	return;
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
APPL_State_t APPL_Get_State(void)
{
	return applState;
}
void APPL_Init(void)
{
	Set_State(APPL_STATE_STARTUP);
}

void APPL_MainFct(void)
{
	APPL_RunStateMachine();
}




#ifdef MASTER_appl_C_
#undef MASTER_appl_C_
#endif /* !MASTER_appl_C_ */
