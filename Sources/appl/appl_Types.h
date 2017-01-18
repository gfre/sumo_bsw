#ifndef APPL_TYPES_H_
#define APPL_TYPES_H_

#include "FRTOS1.h"


#ifdef MASTER_APPL_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


typedef struct APPL_TaskCfgItm_s
{
	TaskFunction_t taskFct;
	const char_t * const taskName;
	const uint16 stackDepth;
	void * const pvParameters;
	uint32 taskPriority;
	TaskHandle_t taskHdl;
}APPL_TaskCfgItm_t;

typedef struct APPL_TaskCfg_s
{
	APPL_TaskCfgItm_t *tasks;
	const uint8 numTasks;
}APPL_TaskCfg_t;

typedef void (APPL_MainFct_t)(void);

typedef struct APPL_MainFctCfg_s
{
	APPL_MainFct_t * const mainFct;
	const char_t * const mainFctName;
}APPL_MainFctCfg_t;

typedef struct APPL_CycTaskFctPar_s
{
	const uint8 taskPeriod;
	const APPL_MainFctCfg_t *mainFctCfg;
	const uint8 numMainFcts;

}APPL_CycTaskFctPar_t;

typedef struct APPL_NonCycTaskFctPar_s
{
	const uint8 taskDelay;
	const APPL_MainFctCfg_t *mainFctCfg;
	const uint8 numMainFcts;

}APPL_NonCycTaskFctPar_t;

EXTERNAL_ void APPL_CycTaskFct(void *pvParameters);
EXTERNAL_ void APPL_NonCycTaskFct(void *pvParameters);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* APPLICATION_H_ */
