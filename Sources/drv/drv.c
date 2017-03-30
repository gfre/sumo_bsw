/*******************************************************************************
 * @brief 	Module to drive the robot.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @date 		06.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module allows to drive the robot and to perform turns.
 *
 * ==============================================================================
 */


#define MASTER_drv_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"
#include "drv.h"
#include "drv_Types.h"
#include "FRTOS1.h"
#include "UTIL1.h"
#include "sh.h"
#include "Tacho.h"
#include "Pid.h"
#include "mot.h"
#include "CLS1.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "WAIT1.h"



/*======================================= >> #DEFINES << =========================================*/
#define PRINT_DRIVE_INFO  	(0) /* if we print debug info */
#define QUEUE_LENGTH      	(4) /* number of items in queue, that's my buffer size */
#define QUEUE_ITEM_SIZE   	(sizeof(DRV_Command)) /* each item is a single drive command */
#define MATCH_MARGIN		(50)
#define DRV_TURN_SPEED_LOW  (50)


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum DRV_Commands_e {
	DRV_SET_MODE,
	DRV_SET_SPEED,
	DRV_SET_POS,
} DRV_Commands;

typedef struct DRV_Command_s {
	DRV_Commands cmd;
	union {
		DRV_Mode mode; /* DRV_SET_MODE */
		struct {
			int32_t left, right;
		} speed; /* DRV_SET_SPEED */
		struct {
			int32_t left, right;
		} pos; /* DRV_SET_POS */
	} u;
} DRV_Command;



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t GetCmd(void);
static bool match(int16_t pos, int16_t target);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static DRV_Status_t DRV_Status;
static xQueueHandle DRV_Queue;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t GetCmd(void) {
	DRV_Command cmd;
	portBASE_TYPE res;

	res = FRTOS1_xQueueReceive(DRV_Queue, &cmd, 0);
	if (res==errQUEUE_EMPTY) {
		return ERR_RXEMPTY; /* no command */
	}
	/* process command */
	FRTOS1_taskENTER_CRITICAL();
	if (cmd.cmd==DRV_SET_MODE) {
		PID_Start(); /* reset PID, especially integral counters */
		DRV_Status.mode = cmd.u.mode;
	} else if (cmd.cmd==DRV_SET_SPEED) {
		DRV_Status.speed.left = cmd.u.speed.left;
		DRV_Status.speed.right = cmd.u.speed.right;
	} else if (cmd.cmd==DRV_SET_POS) {
		DRV_Status.pos.left = cmd.u.pos.left;
		DRV_Status.pos.right = cmd.u.pos.right;
	}
	FRTOS1_taskEXIT_CRITICAL();
#if PRINT_DRIVE_INFO
	{
		uint8_t buf[32];

		if (cmd.cmd==DRV_SET_MODE) {
			UTIL1_strcpy(buf, sizeof(buf), "SETMODE: ");
			UTIL1_strcat(buf, sizeof(buf), DRV_GetModeStr(DRV_Status.mode));
		} else if (cmd.cmd==DRV_SET_SPEED) {
			UTIL1_strcpy(buf, sizeof(buf), "SETSPEED: ");
			UTIL1_strcatNum32s(buf, sizeof(buf), DRV_Status.speed.left);
			UTIL1_strcat(buf, sizeof(buf), ", ");
			UTIL1_strcatNum32s(buf, sizeof(buf), DRV_Status.speed.right);
		} else if (cmd.cmd==DRV_SET_POS) {
			UTIL1_strcpy(buf, sizeof(buf), "SETPOS: ");
			UTIL1_strcatNum32s(buf, sizeof(buf), DRV_Status.pos.left);
			UTIL1_strcat(buf, sizeof(buf), ", ");
			UTIL1_strcatNum32s(buf, sizeof(buf), DRV_Status.pos.right);
		} else {
			UTIL1_strcpy(buf, sizeof(buf), "ERROR!");
		}
		UTIL1_strcat(buf, sizeof(buf), "\r\n");
		SHELL_SendString(buf);
	}
#endif
	return ERR_OK;
}


static bool match(int16_t pos, int16_t target) {
#if MATCH_MARGIN>0
	return (pos>=target-MATCH_MARGIN && pos<=target+MATCH_MARGIN);
#else
	return pos==target;
#endif
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
bool DRV_IsStopped(void) {
	Q4CLeft_QuadCntrType leftPos;
	Q4CRight_QuadCntrType rightPos;

	if (FRTOS1_uxQueueMessagesWaiting(DRV_Queue)>0) {
		return FALSE; /* still messages in command queue, so there is something pending */
	}
	/* do *not* use/calculate speed: too slow! Use position encoder instead */
	leftPos = Q4CLeft_GetPos();
	rightPos = Q4CRight_GetPos();
	if (DRV_Status.mode==DRV_MODE_POS) {
		if (DRV_Status.pos.left!=(int32_t)leftPos) {
			return FALSE;
		}
		if (DRV_Status.pos.right!=(int32_t)rightPos) {
			return FALSE;
		}
		return TRUE;
	} if (DRV_Status.mode==DRV_MODE_STOP) {
		return TRUE;
	} else {
		/* ???? what to do otherwise ???? */
		return FALSE;
	}
}

uint8_t DRV_Stop(int32_t timeoutMs) {
	(void)DRV_SetMode(DRV_MODE_STOP); /* stop it */
	do {
		if (DRV_IsStopped()) {
			break;
		}
		WAIT1_WaitOSms(5);
		timeoutMs -= 5;
	} while (timeoutMs>0);
	if (timeoutMs<0) {
		return ERR_BUSY; /* timeout */
	}
	return ERR_OK;
}

bool DRV_IsDrivingBackward(void) {
	return DRV_Status.mode==DRV_MODE_SPEED
			&& DRV_Status.speed.left<0
			&& DRV_Status.speed.right<0;
}



bool DRV_HasTurned(void) {
	int16_t pos;

	if (FRTOS1_uxQueueMessagesWaiting(DRV_Queue)>0) {
		return FALSE; /* still messages in command queue, so there is something pending */
	}
	if (DRV_Status.mode==DRV_MODE_POS) {
		int32_t speedL, speedR;

		speedL = TACHO_GetSpeed(TRUE);
		speedR = TACHO_GetSpeed(FALSE);
		if (speedL>-DRV_TURN_SPEED_LOW && speedL<DRV_TURN_SPEED_LOW && speedR>-DRV_TURN_SPEED_LOW && speedR<DRV_TURN_SPEED_LOW) { /* speed close to zero */
			pos = Q4CLeft_GetPos();
			if (match(pos, DRV_Status.pos.left)) {
				pos = Q4CRight_GetPos();
				if (match(pos, DRV_Status.pos.right)) {
					return TRUE;
				}
			}
		}
		return FALSE;
	} /* if */
	return TRUE;
}

DRV_Mode DRV_GetMode(void) {
	return DRV_Status.mode;
}

uint8_t DRV_SetMode(DRV_Mode mode) {
	DRV_Command cmd;

#if 1 /*PL_HAS_DRIVE_STOP_POS*/
	if (mode==DRV_MODE_STOP) {
		(void)DRV_SetPos(Q4CLeft_GetPos(), Q4CRight_GetPos()); /* set current position */
		PID_Start(); /* reset PID, especially integral counters */
		mode = DRV_MODE_POS;
	}
#endif
	cmd.cmd = DRV_SET_MODE;
	cmd.u.mode = mode;
	if (FRTOS1_xQueueSendToBack(DRV_Queue, &cmd, portMAX_DELAY)!=pdPASS) {
		return ERR_FAILED;
	}
	FRTOS1_taskYIELD(); /* yield so drive task has a chance to read message */
	return ERR_OK;
}

uint8_t DRV_SetSpeed(int32_t left, int32_t right) {
	DRV_Command cmd;

	cmd.cmd = DRV_SET_SPEED;
	cmd.u.speed.left = left;
	cmd.u.speed.right = right;
	if (FRTOS1_xQueueSendToBack(DRV_Queue, &cmd, portMAX_DELAY)!=pdPASS) {
		return ERR_FAILED;
	}
	FRTOS1_taskYIELD(); /* yield so drive task has a chance to read message */
	return ERR_OK;
}

uint8_t DRV_SetPos(int32_t left, int32_t right) {
	DRV_Command cmd;

	cmd.cmd = DRV_SET_POS;
	cmd.u.pos.left = left;
	cmd.u.pos.right = right;
	if (FRTOS1_xQueueSendToBack(DRV_Queue, &cmd, portMAX_DELAY)!=pdPASS) {
		return ERR_FAILED;
	}
	FRTOS1_taskYIELD(); /* yield so drive task has a chance to read message */
	return ERR_OK;
}

void DRV_DeInit(void) {
	FRTOS1_vQueueDelete(DRV_Queue);
}

void DRV_Init(void) {
	MOT_Init();

	DRV_Status.mode = DRV_MODE_NONE;
	DRV_Status.speed.left = 0;
	DRV_Status.speed.right = 0;
	DRV_Status.pos.left = 0;
	DRV_Status.pos.right = 0;
	DRV_Queue = FRTOS1_xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	if (DRV_Queue==NULL) {
		for(;;){} /* out of memory? */
	}
	FRTOS1_vQueueAddToRegistry(DRV_Queue, "Drive");
	//if (FRTOS1_xTaskCreate(DriveTask, "Drive", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL) != pdPASS) {
	//	for(;;){} /* error */
	//}
}

void DRV_MainFct(void)
{
	while (GetCmd()==ERR_OK) { /* returns ERR_RXEMPTY if queue is empty */
		/* process incoming commands */
	}
	if (DRV_Status.mode==DRV_MODE_SPEED) {
		PID_Speed(TACHO_GetSpeed(TRUE), DRV_Status.speed.left, TRUE);
		PID_Speed(TACHO_GetSpeed(FALSE), DRV_Status.speed.right, FALSE);
	} else if (DRV_Status.mode==DRV_MODE_STOP) {
		PID_Speed(TACHO_GetSpeed(TRUE), 0, TRUE);
		PID_Speed(TACHO_GetSpeed(FALSE), 0, FALSE);
	} else if (DRV_Status.mode==DRV_MODE_POS) {
		PID_Pos(Q4CLeft_GetPos(), DRV_Status.pos.left, TRUE);
		PID_Pos(Q4CRight_GetPos(), DRV_Status.pos.right, FALSE);
	} else if (DRV_Status.mode==DRV_MODE_NONE) {
		/* do nothing */
	}
	return;
}

DRV_Status_t *DRV_GetCurStatus(void)
{
	return &DRV_Status;
}
