/***********************************************************************************************//**
 * @file		drv.c
 * @ingroup		drv
 * @brief 		Implementation of a driver for controlling the movement of the robot.
 *
 * This software component implements a driver for controlling the movement of the robot in
 * the following driving control modes:
 * > - STOP for standstill control
 * > - SPEED for velocity control and
 * > - POSITION control which provides to drive to a certain odometer target value.
 * It decouples the drive control algorithm from the actual application using a @a queue for the
 * communication between the application and this component.\n
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.01.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/


#define MASTER_drv_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "drv.h"
#include "drv_api.h"
#include "pid_api.h"
#include "kf_api.h"
#include "tacho_api.h"
#include "mot.h"
#include "mot_api.h"

#include "FRTOS1.h"
#include "UTIL1.h"
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
typedef enum DRV_Cmd_e {
	DRV_SET_MODE,
	DRV_SET_SPEED,
	DRV_SET_POS,
} DRV_Cmd_t;

typedef struct DRV_Int32_s
{
	int32_t left;
	int32_t right;
} DRV_Int32_t;

typedef struct DRV_Command_s {
	DRV_Cmd_t cmd;
	union {
		DRV_Mode_t mode;    /* DRV_SET_MODE */
		DRV_Int32_t speed;	/* DRV_SET_SPEED */
		DRV_Int32_t pos;		/* DRV_SET_POS */
	};
} DRV_Command;



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t GetCmd(void);
static bool match(int16_t pos, int16_t target);
static void DRV_ParsePIValToMotor(int32_t PIVal_, bool isLeft_);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static DRV_Status_t DRV_Status;
static xQueueHandle DRV_Queue;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static StdRtn_t GetCmd(void) {
	DRV_Command cmd = {0u};
	StdRtn_t retVal = ERR_OK;

	if (errQUEUE_EMPTY == FRTOS1_xQueueReceive(DRV_Queue, &cmd, 0)) {
		retVal = ERR_RXEMPTY; /* no command */
	}
	else
	{
		/* process command */
			FRTOS1_taskENTER_CRITICAL();
			if (cmd.cmd==DRV_SET_MODE) {
//				PID_Start(); /* reset PID, especially integral counters */
				Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD].Saturation = 0;
				Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD].integralVal = 0;
				Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD].lastError = 0;

				Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD].Saturation = 0;
				Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD].integralVal = 0;
				Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD].lastError = 0;

				Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS].Saturation = 0;
				Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS].integralVal = 0;
				Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS].lastError = 0;

				Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS].Saturation = 0;
				Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS].integralVal = 0;
				Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS].lastError = 0;
				DRV_Status.mode = cmd.mode;
			} else if (cmd.cmd==DRV_SET_SPEED) {
				DRV_Status.speed.left = cmd.speed.left;
				DRV_Status.speed.right = cmd.speed.right;
			} else if (cmd.cmd==DRV_SET_POS) {
				DRV_Status.pos.left = cmd.pos.left;
				DRV_Status.pos.right = cmd.pos.right;
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
	}

	return retVal;
}


static bool match(int16_t pos, int16_t target) {
#if MATCH_MARGIN>0
	return (pos>=target-MATCH_MARGIN && pos<=target+MATCH_MARGIN);
#else
	return pos==target;
#endif
}
static void DRV_ParsePIValToMotor(int32_t PIVal_, bool isLeft_)
{
	MOT_Direction_t direction = MOT_DIR_FORWARD;
	MOT_MotorDevice_t *motHandle;
	if (PIVal_ >= 0)
	{
		direction = MOT_DIR_FORWARD;
	}
	else /* negative, make it positive */
	{
		PIVal_ = -PIVal_; /* make positive */
		direction = MOT_DIR_BACKWARD;
	}
	if(TRUE == isLeft_) motHandle = MOT_GetMotorHandle(MOT_MOTOR_LEFT);
	else				motHandle = MOT_GetMotorHandle(MOT_MOTOR_RIGHT);
	if(NULL != motHandle)
	{
		MOT_SetVal(motHandle, 0xFFFF-PIVal_); /* PWM is low active */
		MOT_SetDirection(motHandle, direction);
		MOT_UpdatePercent(motHandle, direction);
	}
	else
	{
		/* error handling */
	}
}
/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
uint8_t DRV_SetMode(DRV_Mode_t mode) {
	DRV_Command cmd;
	uint8_t i = 0u;
	if (mode==DRV_MODE_STOP) {
		(void)DRV_SetPos(Q4CLeft_GetPos(), Q4CRight_GetPos()); /* set current position */
		//PID_Start(); /* reset PID, especially integral counters */
		Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD].Saturation = 0;
		Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD].integralVal = 0;
		Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD].lastError = 0;

		Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD].Saturation = 0;
		Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD].integralVal = 0;
		Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD].lastError = 0;

		Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS].Saturation = 0;
		Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS].integralVal = 0;
		Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS].lastError = 0;

		Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS].Saturation = 0;
		Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS].integralVal = 0;
		Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS].lastError = 0;
		mode = DRV_MODE_POS;
	}

	cmd.cmd = DRV_SET_MODE;
	cmd.mode = mode;
	if (FRTOS1_xQueueSendToBack(DRV_Queue, &cmd, portMAX_DELAY)!=pdPASS) {
		return ERR_FAILED;
	}
	FRTOS1_taskYIELD(); /* yield so drive task has a chance to read message */
	return ERR_OK;
}

uint8_t DRV_SetSpeed(int32_t left, int32_t right) {
	DRV_Command cmd;

	cmd.cmd = DRV_SET_SPEED;
	cmd.speed.left = left;
	cmd.speed.right = right;
	if (FRTOS1_xQueueSendToBack(DRV_Queue, &cmd, portMAX_DELAY)!=pdPASS) {
		return ERR_FAILED;
	}
	FRTOS1_taskYIELD(); /* yield so drive task has a chance to read message */
	return ERR_OK;
}

uint8_t DRV_SetPos(int32_t left, int32_t right) {
	DRV_Command cmd;

	cmd.cmd = DRV_SET_POS;
	cmd.pos.left = left;
	cmd.pos.right = right;
	if (FRTOS1_xQueueSendToBack(DRV_Queue, &cmd, portMAX_DELAY)!=pdPASS) {
		return ERR_FAILED;
	}
	FRTOS1_taskYIELD(); /* yield so drive task has a chance to read message */
	return ERR_OK;
}


DRV_Mode_t DRV_GetMode(void) {
	return DRV_Status.mode;
}

bool DRV_IsStopped(void) {
	Q4CLeft_QuadCntrType leftPos;
	Q4CRight_QuadCntrType rightPos;

	if (FRTOS1_uxQueueMessagesWaiting(DRV_Queue)>0) {
		return FALSE; /* still messages in command queue, so there is something pending */
	}
	/* do *not* use/calculate speed: too slow! Use position encoder instead */
	leftPos  = Q4CLeft_GetPos();
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

		speedL = Get_pTachoCfg()->pFilterTable[TACHO_Get_FltrType()].pGetSpeedFct(TRUE);
		speedR = Get_pTachoCfg()->pFilterTable[TACHO_Get_FltrType()].pGetSpeedFct(FALSE);
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


void DRV_DeInit(void) {
	FRTOS1_vQueueDelete(DRV_Queue);
}

void DRV_Init(void) {
	uint8_t i = 0u;
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
}

void DRV_MainFct(void)
{
	StdRtn_t retVal = ERR_OK;
	int32_t PIDVal = 0;
	while (GetCmd()==ERR_OK)  /* returns ERR_RXEMPTY if queue is empty */
	{
		/* process incoming commands */
	}

	if (DRV_Status.mode==DRV_MODE_SPEED)
	{
		retVal |= PID( &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD]), &PIDVal);
		DRV_ParsePIValToMotor(PIDVal, TRUE);

		retVal |= PID( &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD]), &PIDVal);
		DRV_ParsePIValToMotor(PIDVal, FALSE);
	}
	else if (DRV_Status.mode==DRV_MODE_STOP)
	{
		DRV_SetSpeed(0, 0);

		retVal |= PID( &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_SPD]), &PIDVal);
		DRV_ParsePIValToMotor(PIDVal, TRUE);

		retVal |= PID( &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_SPD]), &PIDVal);
		DRV_ParsePIValToMotor(PIDVal, FALSE);
	}
	else if (DRV_Status.mode==DRV_MODE_POS)
	{
		retVal |= PID( &(Get_pPidCfg()->pItmTbl[PID_LFT_MTR_POS]), &PIDVal );
		PIDVal = PIDVal*50;
		DRV_ParsePIValToMotor(PIDVal, TRUE);

		retVal |= PID( &(Get_pPidCfg()->pItmTbl[PID_RGHT_MTR_POS]), &PIDVal );
		PIDVal = PIDVal*50;
		DRV_ParsePIValToMotor(PIDVal, FALSE);
	}
	else if (DRV_Status.mode==DRV_MODE_NONE)
	{
		/* do nothing */
	}
	return;
}

DRV_Status_t *DRV_GetCurStatus(void)
{
	return &DRV_Status;
}

StdRtn_t DRV_Read_LftSpdTrgtVal(int32_t* speed_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != speed_)
	{
		*speed_ = DRV_Status.speed.left;
		retVal 	= ERR_OK;
	}
	return retVal;
}

StdRtn_t DRV_Read_RghtSpdTrgtVal(int32_t* speed_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != speed_)
	{
		*speed_ = DRV_Status.speed.right;
		retVal 	= ERR_OK;
	}
	return retVal;
}

StdRtn_t DRV_Read_LftPosTrgtVal(int32_t* pos_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != pos_)
	{
		*pos_ = DRV_Status.pos.left;
		retVal  = ERR_OK;
	}
	return retVal;
}

StdRtn_t DRV_Read_RghtPosTrgtVal(int32_t* pos_)
{
	StdRtn_t retVal = ERR_PARAM_ADDRESS;
	if(NULL != pos_)
	{
		*pos_ = DRV_Status.pos.right;
		retVal 	= ERR_OK;
	}
	return retVal;
}



