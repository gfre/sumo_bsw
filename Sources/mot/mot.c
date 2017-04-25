/***********************************************************************************************//**
 * @file		mot.c
 * @ingroup		mot
 * @brief 		Implementation of the DC motor driver.
 *
 * This software component implements a  driver for up to two small DC motors. It uses @a PWM and
 * @a BitIO firmware components from Kinets to influence the speed and direction of the motors.
 * The driver can handle inverted polarity from assembly point of view.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	06.01.2017
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_motor_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "mot.h"
#include "mot_api.h"
#include "DIRR.h"
#include "DIRL.h"
#include "PWMR.h"
#include "PWML.h"



/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
static uint8_t PWMLSetRatio16(uint16_t ratio);
static uint8_t PWMRSetRatio16(uint16_t ratio);
static void DirLPutVal(bool val);
static void DirRPutVal(bool val);



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static MOT_MotorDevice motorL, motorR;
static bool isMotorOn = TRUE;



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
static uint8_t PWMLSetRatio16(uint16_t ratio) {
	return PWML_SetRatio16(ratio);
}

static uint8_t PWMRSetRatio16(uint16_t ratio) {
	return PWMR_SetRatio16(ratio);
}

static void DirLPutVal(bool val) {
	DIRL_PutVal(val);
}

static void DirRPutVal(bool val) {
	DIRR_PutVal(val);
}



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
MOT_MotorDevice *MOT_GetMotorHandle(MOT_MotorSide side) {
	if (side==MOT_MOTOR_LEFT) {
		return &motorL;
	} else {
		return &motorR;
	}
}


void MOT_SetVal(MOT_MotorDevice *motor, uint16_t val) {
	if (isMotorOn) {
		motor->currPWMvalue = val;
		(void)motor->SetRatio16(val);
	} else { /* have motor stopped */
		motor->currPWMvalue = 0xFFFF;
		(void)motor->SetRatio16(0xFFFF);
	}
}

void MOT_OnOff(bool on) {
	isMotorOn = on;
	if (!on) {
		MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT), 0);
		MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), 0);
	}
}

uint16_t MOT_GetVal(MOT_MotorDevice *motor) {
	return motor->currPWMvalue;
}

void MOT_Invert(MOT_MotorDevice *motor, bool inverted) {
	motor->inverted = inverted;
}

void MOT_SetSpeedPercent(MOT_MotorDevice *motor, MOT_SpeedPercent percent) {
	uint32_t val;

	if (percent>100) { /* make sure we are within 0..100 */
		percent = 100;
	} else if (percent<-100) {
		percent = -100;
	}
	motor->currSpeedPercent = percent; /* store value */
	if (percent<0) {
		MOT_SetDirection(motor, MOT_DIR_BACKWARD);
		percent = -percent; /* make it positive */
	} else {
		MOT_SetDirection(motor, MOT_DIR_FORWARD);
	}
	val = ((100-percent)*0xffff)/100; /* H-Bridge is low active! */
	MOT_SetVal(motor, (uint16_t)val);
}

void MOT_UpdatePercent(MOT_MotorDevice *motor, MOT_Direction dir) {
	motor->currSpeedPercent = ((0xffff-motor->currPWMvalue)*100)/0xffff;
	if (dir==MOT_DIR_BACKWARD) {
		motor->currSpeedPercent = -motor->currSpeedPercent;
	}
}

MOT_SpeedPercent MOT_GetSpeedPercent(MOT_MotorDevice *motor) {
	return motor->currSpeedPercent;
}

void MOT_ChangeSpeedPercent(MOT_MotorDevice *motor, MOT_SpeedPercent relPercent) {
	relPercent += motor->currSpeedPercent; /* make absolute number */
	if (relPercent>100) { /* check for overflow */
		relPercent = 100;
	} else if (relPercent<-100) { /* and underflow */
		relPercent = -100;
	}
	MOT_SetSpeedPercent(motor, relPercent);  /* set speed. This will care about the direction too */
}

void MOT_SetDirection(MOT_MotorDevice *motor, MOT_Direction dir) {
	if (dir==MOT_DIR_FORWARD ) {
		motor->DirPutVal(motor->inverted?0:1);
		if (motor->currSpeedPercent<0) {
			motor->currSpeedPercent = -motor->currSpeedPercent;
		}
	} else if (dir==MOT_DIR_BACKWARD) {
		motor->DirPutVal(motor->inverted?1:0);
		if (motor->currSpeedPercent>0) {
			motor->currSpeedPercent = -motor->currSpeedPercent;
		}
	}
}

MOT_Direction MOT_GetDirection(MOT_MotorDevice *motor) {
	if (motor->currSpeedPercent<0) {
		return MOT_DIR_BACKWARD;
	} else {
		return MOT_DIR_FORWARD;
	}
}

uint8_t MOT_Get_IsMotorOn(void)
{
	return (uint8_t)isMotorOn;
}


void MOT_Deinit(void) {
	/* nothig needed for now */
}

void MOT_Init(void) {
	motorL.inverted = CAU_SUMO_PLT_MOTOR_LEFT_INVERTED;
	motorR.inverted = CAU_SUMO_PLT_MOTOR_RIGHT_INVERTED;
	motorL.DirPutVal = DirLPutVal;
	motorR.DirPutVal = DirRPutVal;
	motorL.SetRatio16 = PWMLSetRatio16;
	motorR.SetRatio16 = PWMRSetRatio16;
	MOT_SetSpeedPercent(&motorL, 0);
	MOT_SetSpeedPercent(&motorR, 0);
	(void)PWML_Enable();
	(void)PWMR_Enable();
	isMotorOn = TRUE;
}

