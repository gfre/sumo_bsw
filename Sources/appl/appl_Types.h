#ifndef APPL_TYPES_H_
#define APPL_TYPES_H_

#include "FRTOS1.h"

#ifdef MASTER_APPL_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


typedef struct TaskCfg_s
{
	TaskFunction_t taskFct;
	const char_t * const taskName;
	const uint16 stackDepth;
	void * const pvParameters;
	uint32 taskPriority;
	TaskHandle_t * const taskHdl;
}TaskCfg_t;

typedef struct APPL_TaskCfg_s
{
	const TaskCfg_t *tasks;
	uint8 numTasks;
}APPL_TaskCfg_t;

typedef void (*MainFctHl_t)(void);

typedef struct APPL_cycTaskFctPar_s
{
	uint8 taskPeriod;
	MainFctHl_t *mainFcts;
	uint8 numMainFcts;

}APPL_cycTaskFctPar_t;

typedef struct APPL_nonCycTaskFctPar_s
{
	uint8 taskDelay;
	MainFctHl_t *mainFcts;
	uint8 numMainFcts;

}APPL_nonCycTaskFctPar_t;

EXTERNAL_ void APPL_cycTaskFct(void *pvParameters);
EXTERNAL_ void APPL_nonCycTaskFct(void *pvParameters);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* APPLICATION_H_ */
