/***********************************************************************************************//**
 * @file		mot_api.h
 * @ingroup		mot
 * @brief 		API of the SWC @a Motor Driver
 *
 * This API provides a BSW-internal interface of the SWC @ref mot. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	25.04.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#ifndef MOT_API_H_
#define MOT_API_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_mot_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup mot
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum {
	MOT_DIR_FORWARD,  /*!< Motor forward direction */
	MOT_DIR_BACKWARD  /*!< Motor backward direction */
} MOT_Direction;

typedef int8_t MOT_SpeedPercent; 		/*!< -100%...+100%, where negative is backward */

typedef struct MOT_MotorDevice_ {
	bool inverted;
	MOT_SpeedPercent currSpeedPercent;	/*!< our current speed in %, negative percent means backward */
	uint16_t currPWMvalue; 				/*!< current PWM value used */
	uint8_t (*SetRatio16)(uint16_t); 	/*!< function to set the ratio */
	void (*DirPutVal)(bool); 			/*!< function to set direction bit */

} MOT_MotorDevice;

typedef enum {
	MOT_MOTOR_LEFT, /*!< left motor */
	MOT_MOTOR_RIGHT /*!< right motor */
} MOT_MotorSide;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Inverts the forward/backward signal for a motor
 * @param motor Motor handle
 * @param invert TRUE to invert logic, FALSE otherwise
 */
void MOT_Invert(MOT_MotorDevice *motor, bool invert);

/**
 * @brief Function to get a pointer to a motor (motor handle)
 * @param side Which motor
 * @return Pointer/handle to the motor
 */
MOT_MotorDevice *MOT_GetMotorHandle(MOT_MotorSide side);

/**
 * @brief Changes the speed of a motor, in the range of -100% (backward) to +100% (forward).
 * @param motor Motor handle.
 * @param relPercent Relative speed percentage to change.
 */
void MOT_ChangeSpeedPercent(MOT_MotorDevice *motor, MOT_SpeedPercent relPercent);

/**
 * @brief Returns the speed for a motor in percent (negative values are backward, positive are forward
 * @param motor Motor handle
 * @return The speed in percent, in the range -100...100
 */
MOT_SpeedPercent MOT_GetSpeedPercent(MOT_MotorDevice *motor);

/**
 * @brief Sets the speed of a motor, in the range of -100% (backward) to +100% (forward).
 * @param motor Motor handle.
 * @param percent Motor speed value, from -100 (full speed backward) to +100 (full speed forward).
 */
void MOT_SetSpeedPercent(MOT_MotorDevice *motor, MOT_SpeedPercent percent);

/**
 * @brief Updates the motor % speed based on actual PWM value.
 * @param motor Motor handle.
 * @param dir Current direction of motor.
 */
void MOT_UpdatePercent(MOT_MotorDevice *motor, MOT_Direction dir);

/**
 * @brief Sets the PWM value for the motor.
 * @param[in] motor Motor handle
 * @param[in] val New PWM value.
 */
void MOT_SetVal(MOT_MotorDevice *motor, uint16_t val);

/**
 * @brief Return the current PWM value of the motor.
 * @param[in] motor Motor handle
 * @return Current PWM value.
 */
uint16_t MOT_GetVal(MOT_MotorDevice *motor);

/**
 * @brief Change the direction of the motor
 * @param[in] motor Motor handle
 * @param[in] dir Direction to be used
 */
void MOT_SetDirection(MOT_MotorDevice *motor, MOT_Direction dir);

/**
 * @brief Returns the direction of the motor
 * @param[in] motor Motor handle
 * @return Current direction of the motor
 */
MOT_Direction MOT_GetDirection(MOT_MotorDevice *motor);

/**
 * @brief Function to turn motors on/off, useful for debugging
 * @param on If motors shall be turned on or off. If turning off, the PWM is set to the 'off' level too.
 */
void MOT_OnOff(bool on);

/*!
 * @brief Function returns motor state ON or OFF
 * @return motor state ON or OFF
 */
uint8_t MOT_Get_IsMotorOn(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !MOT_API_H_ */
