/***************************************************************************************************
 * @brief 	State machine for handling application software
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This software component implements an extended state machine which handles
 * - startup procedures
 * - initialisations of application software without its own task
 * - custom application software in IDLE and NORMAL mode
 * - debug state for debugging basic software on command line shell
 * - an ERROR state for error handling
 *==================================================================================================
 */

#define MASTER_appl_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "appl.h"
#include "appl_Types.h"
#include "Platform.h"
#include "stud.h"
#include "task_cfg.h"
#include "rte.h"
#include "sh.h"
#include "buz.h"
#include "batt.h"
#include "id.h"
#include "nvm.h"
#include "rte_Types.h"
#include "Pid.h"
#ifdef ASW_ENABLED
#include "asw.h"
#endif



/*======================================= >> #DEFINES << =========================================*/
#define CLR_HOLD_ON_BIT(holdOn_, state_)				( (uint8_t)( holdOn_ & ~( 0x01u << state_ ) ) )
#define SET_HOLD_ON_BIT(holdOn_, state_)				( (uint8_t)( holdOn_ |  ( 0x01u << state_ ) ) )
#define GET_HOLD_ON_BIT(holdOn_, state_)				( (uint8_t)( 0x01u   &  ( holdOn_ >> state_ ) ) )
#define CHK_HOLD_ON_BIT(holdOn_ , release_, state_)		( TRUE == GET_HOLD_ON_BIT(holdOn_, state_) ? GET_HOLD_ON_BIT(release_, state_) : TRUE)

#define CHK_HOLD_ON_ENTER(state_)						(CHK_HOLD_ON_BIT(holdOnEnter, releaseEnter, state_))
#define CHK_HOLD_ON_EXIT(state_)						(CHK_HOLD_ON_BIT(holdOnExit, releaseExit, state_))
#define CLR_HOLD_ON_ENTER(state_)						(CLR_HOLD_ON_BIT(releaseEnter, state_))
#define CLR_HOLD_ON_EXIT(state_)						(CLR_HOLD_ON_BIT(releaseExit, state_))

/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct SmType_s
{
	APPL_State_t state;
	APPL_Cmd_t cmd;
} SmType_t;

typedef StdRtn_t (SmFct_t)(void);

typedef struct SmStFcts_s
{
	SmFct_t *enterFct;
	SmFct_t *runFct;
	SmFct_t *exitFct;
} SmStFcts_t;

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static StdRtn_t runSTARTUP(void);
static StdRtn_t runINIT(void);

static StdRtn_t runIDLE(void);
static StdRtn_t exitIDLE(void);

static StdRtn_t enterNORMAL(void);
static StdRtn_t runNORMAL(void);
static StdRtn_t exitNORMAL(void);

static StdRtn_t enterDEBUG(void);
static StdRtn_t runDEBUG(void);
static StdRtn_t exitDEBUG(void);

static StdRtn_t enterERROR(void);
static StdRtn_t runERROR(void);

static void Tick_StateMachine(void);
static inline void Proc_States(const SmType_t sm_);
static inline StdRtn_t Sync_StateMachineWithISR();

static inline StdRtn_t Set_HoldOnMask(uint8_t *mask_, const APPL_State_t state_, const uint8_t holdOn_);

/*=================================== >> GLOBAL VARIABLES << =====================================*/
const TASK_CfgItm_t *dbgTaskCfg = NULL;
static SmType_t sm = {APPL_STATE_NONE, noCmd};
static APPL_State_t nextState = APPL_STATE_NONE;
static SmStFcts_t smStFctTbl[APPL_STATE_NUM] = {
/* STARTUP */ 	{NULL, 			runSTARTUP, NULL},
/* INIT */ 		{NULL, 			runINIT, 	NULL},
/* IDLE */		{NULL, 			runIDLE, 	exitIDLE},
/* NORMAL */	{enterNORMAL,	runNORMAL, 	exitNORMAL},
/* DEBUG */		{enterDEBUG, 	runDEBUG, 	exitDEBUG},
/* ERROR */		{enterERROR, 	runERROR, 	NULL},
};

static uint8_t holdOnEnter  = 0x00u;
static uint8_t holdOnExit   = 0x00u;
static uint8_t releaseEnter = 0x00u;
static uint8_t releaseExit  = 0x00u;
/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static inline void Proc_States(const SmType_t sm_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	switch(sm_.cmd)
	{
		case Enter:
		{
			if ( ( NULL != smStFctTbl[sm_.state].enterFct ) )
			{
				retVal = smStFctTbl[sm_.state].enterFct();
			}
			break;
		}
		case Run:
		{
			if ( NULL != smStFctTbl[sm_.state].runFct )
			{
				retVal = smStFctTbl[sm_.state].runFct();
			}
			break;
		}
		case Exit:
		{
			if ( NULL != smStFctTbl[sm_.state].exitFct )
			{
				retVal = smStFctTbl[sm_.state].exitFct();
			}
			break;
		}
		default:
		{
			(void)runERROR();
			break;
		}
	}
}


static void Tick_StateMachine(void)
{
   if ( ( sm.state != APPL_STATE_NONE )  &&  ( sm.cmd != noCmd ) )
   {
	   Proc_States(sm);
	   if ( nextState != APPL_STATE_NONE)
	   {
		   if( ( Exit == sm.cmd ) && ( TRUE == CHK_HOLD_ON_EXIT(sm.state) ) )
		   {
			   CLR_HOLD_ON_EXIT(sm.state);
			   sm.cmd = Enter;
			   sm.state = nextState;
			   nextState = APPL_STATE_NONE;
		   }
		   else
		   {
			   sm.cmd = Exit;
		   }
	   }
	   else
	   {
		   if( ( Enter == sm.cmd ) && ( TRUE == CHK_HOLD_ON_ENTER(sm.state) ) )
		   {
			   CLR_HOLD_ON_ENTER(sm.state);
			   sm.cmd = Run;
		   }
	   }

   }
   else
   {
	   sm.state = APPL_STATE_ERROR;
	   sm.cmd = Enter;
   }
   Sync_StateMachineWithISR();
}


static inline StdRtn_t Sync_StateMachineWithISR()
{
  BaseType_t notfRes = pdFAIL;
  uint32_t notfVal = 0u;

  notfRes = FRTOS1_xTaskNotifyWait( pdFALSE,
				    UINT32_MAX,
				    (uint32_t *)&notfVal,
				    pdMS_TO_TICKS( 0u ) );

  /* Transitions from IDLE state */
  if( ( pdPASS == notfRes ) && ( APPL_STATE_IDLE == sm.state ) )
  {
      /* Handle transition from IDLE --> NORMAL */
      if( (notfVal & KEY_RELEASED_NOTIFICATION_VALUE) != FALSE)
      {
    	  nextState = APPL_STATE_NORMAL;
      }
      /* Handle transition from IDLE --> DEBUG */
      if( (notfVal & KEY_PRESSED_LONG_NOTIFICATION_VALUE) != FALSE)
      {
    	  nextState = APPL_STATE_DEBUG;
      }
  }
  /* Transitions from DEBUG state */
  else if( ( pdPASS == notfRes ) && ( APPL_STATE_DEBUG == sm.state ) )
  {
      /* Handle transition from DEBUG --> IDLE */
      if( (notfVal & KEY_PRESSED_LONG_NOTIFICATION_VALUE) != FALSE)
      {
    	  nextState = APPL_STATE_IDLE;
      }
  }
  else
  {
	  /* do nothing */
  }
}

static StdRtn_t runSTARTUP(void)
{
	NVM_Init();
	PID_Init();
	RTE_Init();
	nextState = APPL_STATE_INIT;
	return ERR_OK;
}

static StdRtn_t runINIT(void)
{
	dbgTaskCfg = TASK_Get_DbgTaskCfg();
	ID_Init();
	BUZ_Init();
	BATT_Init();
	STUD_Init();
#ifdef ASW_ENABLED
	ASW_Init();
#endif
	nextState = APPL_STATE_IDLE;
    return ERR_OK;
}

static StdRtn_t runIDLE(void)
{
	return ERR_OK;
}


static StdRtn_t exitIDLE(void)
{
	return RTE_Write_BuzPlayTune(BUZ_TUNE_BUTTON);
}

static StdRtn_t enterNORMAL(void)
{
	return ERR_OK;
}

static StdRtn_t runNORMAL(void)
{
#ifdef ASW_ENABLED
	ASW_Main();
#endif
  STUD_Main();

  return ERR_OK;
}

static StdRtn_t exitNORMAL(void)
{
	return ERR_OK;
}


static StdRtn_t enterDEBUG(void)
{
	SH_Init();
	FRTOS1_vTaskResume(dbgTaskCfg->taskHdl);
	return RTE_Write_BuzPlayTune(BUZ_TUNE_ACCEPT);
}

static StdRtn_t runDEBUG(void)
{
  return ERR_OK;
}

static StdRtn_t exitDEBUG(void)
{
	SH_Deinit();
	FRTOS1_vTaskSuspend(dbgTaskCfg->taskHdl);
	return RTE_Write_BuzPlayTune(BUZ_TUNE_DECLINE);
}

static StdRtn_t enterERROR(void)
{
	return ERR_OK;
}

static StdRtn_t runERROR(void)
{
	return ERR_OK;
}

static inline StdRtn_t Set_HoldOnMask(uint8_t *mask_, const APPL_State_t state_, const uint8_t holdOn_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if (NULL != mask_)
	{
		retVal = ERR_OK;
		if ( FALSE == holdOn_)
		{
			*mask_ = CLR_HOLD_ON_BIT(*mask_,(uint8_t)state_);
		}
		else
		{
			*mask_ = SET_HOLD_ON_BIT(*mask_,(uint8_t)state_);
		}
	}

	return retVal;
}


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
APPL_State_t APPL_Get_NextState(void)
{
	return nextState;
}

APPL_State_t APPL_Get_SmState(void)
{
	return sm.state;
}

APPL_Cmd_t APPL_Get_SmCmd(void)
{
	return sm.cmd;
}

StdRtn_t Set_HoldOnEnter(const APPL_State_t state_, const uint8_t holdOn_)
{
	return Set_HoldOnMask(&holdOnEnter, state_, holdOn_);
}

StdRtn_t Set_HoldOnExit(const APPL_State_t state_, const uint8_t holdOn_)
{
	return Set_HoldOnMask(&holdOnExit, state_, holdOn_);
}

StdRtn_t Set_ReleaseEnter(const APPL_State_t state_)
{
	return Set_HoldOnMask(&releaseEnter, state_, TRUE);
}

StdRtn_t Set_ReleaseExit(const APPL_State_t state_)
{
	return Set_HoldOnMask(&releaseExit, state_,TRUE);
}

void APPL_Init(void)
{
	sm.state = APPL_STATE_STARTUP;
	sm.cmd = Enter;
	holdOnEnter = 0x00u;
	holdOnExit  = 0x00u;
}

void APPL_MainFct(void)
{
	Tick_StateMachine();
}




#ifdef MASTER_appl_C_
#undef MASTER_appl_C_
#endif /* !MASTER_appl_C_ */
