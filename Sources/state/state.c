#include "Platform.h"
#include "state.h"

static AppStateType appState = APP_STATE_STARTUP;

void StateMachine(void) {
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
