#include "Platform.h"
#include "appl.h"
#include "FRTOS1.h"
#include "WAIT1.h"
#include "Shell.h"
#include "LED1.h"
#include "LED2.h"
#include "Buzzer.h"
#include "KEY1.h"
#include "Motor.h"
#include "RNET1.h"
#include "batt.h"
#include "Tacho.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "Pid.h"
#include "Drive.h"

typedef enum AppStateType_s{
	APP_STATE_STARTUP,
	APP_STATE_INIT,
	APP_STATE_IDLE
} AppStateType;

static AppStateType appState = APP_STATE_STARTUP;

void APP_DebugPrint(unsigned char *str) {
	SHELL_SendString(str);
}


static unsigned char *AppStateString(AppStateType state) {
	switch(state) {
	case APP_STATE_STARTUP: return (unsigned char*)"STARTUP";
	case APP_STATE_INIT: return (unsigned char*)"INIT";
	case APP_STATE_IDLE: return (unsigned char*)"IDLE";
	default:
		break;
	}
	return (unsigned char*)"unknown?";
}


static void StateMachine(bool buttonPress) {
	switch (appState) {
	case APP_STATE_STARTUP:
		appState = APP_STATE_INIT;
		break;

	case APP_STATE_INIT:
		RNET1_PowerUp();
		appState = APP_STATE_IDLE;
		break;

	case APP_STATE_IDLE:
		break;

	}/* switch */
	return;
}


static void MainTask(void *pvParameters) {
	(void)pvParameters; /* not used */
	FRTOS1_vTaskDelay(100/portTICK_PERIOD_MS); /* provide some time to get hardware (SW1) pull-up effective */
	for(;;) {
		KEY1_ScanKeys();
		TACHO_CalcSpeed();
		StateMachine(FALSE);
		FRTOS1_vTaskDelay(10/portTICK_PERIOD_MS);
	} /* for */
}


static uint8_t APP_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*)"app", (unsigned char*)"Group of app commands\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows app help or status\r\n", io->stdOut);
	return ERR_OK;
}

static uint8_t APP_PrintStatus(const CLS1_StdIOType *io) {
	CLS1_SendStatusStr((unsigned char*)"app", (unsigned char*)"\r\n", io->stdOut);
	CLS1_SendStatusStr((unsigned char*)"  App State", AppStateString(appState), io->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);
	return ERR_OK;
}

uint8_t APP_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
	uint8_t res = ERR_OK;
	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"app help")==0) {
		*handled = TRUE;
		return APP_PrintHelp(io);
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"app status")==0) {
		*handled = TRUE;
		return APP_PrintStatus(io);
	}
	return res;
}

static void APP_AdoptToHardware(void) {
	/*Possibility to swap Pins*/
	(void)Q4CLeft_SwapPins(TRUE);
	//(void)Q4CRight_SwapPins(TRUE);

	/* SW1: enable and turn on pull-up resistor for PTA14 (push button) */
	PORT_PDD_SetPinPullSelect(PORTA_BASE_PTR, 14, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTA_BASE_PTR, 14, PORT_PDD_PULL_ENABLE);

	/* pull-ups for Quadrature Encoder Pins */
	PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 10, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 10, PORT_PDD_PULL_ENABLE);
	PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 11, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 11, PORT_PDD_PULL_ENABLE);
	PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 16, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 16, PORT_PDD_PULL_ENABLE);
	PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 17, PORT_PDD_PULL_UP);
	PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 17, PORT_PDD_PULL_ENABLE);
}

void APP_Run(void) {
	appState = APP_STATE_STARTUP;
	/*	The following initializations are done by Cpu.c::
	 * KEY1_Init();
	 * LED1_Init();
	 * LED2_Init();
	 * Q4CLeft_Init();
	 * Q4CRight_Init();
	 * TRG1_Init();
	 * PTA_Init();
	 * RTT1_Init();
	 * CLS1_Init();
	 * */
	SHELL_Init();
	BUZ_Init();
	MOT_Init();
	RNET1_Init();
	BATT_Init();	
	TACHO_Init();
	PID_Init();
	DRV_Init();

	APP_AdoptToHardware();
	if (FRTOS1_xTaskCreate(MainTask, "Main", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
		for(;;){} /* error */
	}
	FRTOS1_vTaskStartScheduler();
}
