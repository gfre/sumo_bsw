#include "Platform.h"

typedef enum AppStateType_s{
	APP_STATE_STARTUP,
	APP_STATE_INIT,
	APP_STATE_IDLE
} AppStateType;

void StateMachine(void);
