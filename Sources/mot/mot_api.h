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
#include "Platform.h"


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
/**
 * @brief Data type definition for motor speed values in percent
 */
typedef int8_t MOT_SpeedPercent; 		/*!< -100%...+100%, where negative is backward */

/**
 * @typedef MOT_MotorDevice_t
 * @brief Data type definition of the structure MOT_MotorDevice_s
 *
 * @struct MOT_MotorDevice_s
 * @brief This struct defines the feature of a motor device
 */
typedef struct MOT_MotorDevice_s {
	bool inverted;
	MOT_SpeedPercent currSpeedPercent;	/**< current speed in %, negative values mean backward direction */
	uint16_t currPWMvalue; 				/**< PWM value currently used */
	uint8_t (*SetRatio16)(uint16_t); 	/**< function to set the ratio */
	void (*DirPutVal)(bool); 			/**< function to set the direction bit */

} MOT_MotorDevice_t;

/**
 * @typedef MOT_Direction_t
 * @brief Data type definition of the enumeration MOT_Direction_e
 *
 * @enum MOT_Direction_e
 * @brief This enumeration the driving direction property
 * forward and backward of the robots a unique identification value
 */
typedef enum MOT_Direction_e {
	MOT_DIR_FORWARD,  /*!< Motor forward direction */ //!< MOT_DIR_FORWARD
	MOT_DIR_BACKWARD  /*!< Motor backward direction *///!< MOT_DIR_BACKWARD
} MOT_Direction_t;

/**
 * @typedef MOT_MotorSide_t
 * @brief Data type definition of the enumeration MOT_MotorSide_e
 *
 * @enum MOT_MotorSide_e
 * @brief This enumeration gives the two motors a unique identification using
 * the natural assignment property left- and right-hand side.
 */
typedef enum MOT_MotorSide_e {
	MOT_MOTOR_LEFT, /*!< left motor */
	MOT_MOTOR_RIGHT /*!< right motor */
} MOT_MotorSide_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Inverts the forward/backward signal for a motor
 * @param motor Motor handle
 * @param invert TRUE to invert logic, FALSE otherwise
 */
void MOT_Invert(MOT_MotorDevice_t *motor, bool invert);

/**
 * @brief Function to get a pointer to a motor (motor handle)
 * @param side Which motor
 * @return Pointer/handle to the motor
 */
MOT_MotorDevice_t *MOT_GetMotorHandle(MOT_MotorSide_t side);

/**
 * @brief Changes the speed of a motor, in the range of -100% (backward) to +100% (forward).
 * @param motor Motor handle.
 * @param relPercent Relative speed percentage to change.
 */
void MOT_ChangeSpeedPercent(MOT_MotorDevice_t *motor, MOT_SpeedPercent relPercent);

/**
 * @brief Returns the speed for a motor in percent (negative values are backward, positive are forward
 * @param motor Motor handle
 * @return The speed in percent, in the range -100...100
 */
MOT_SpeedPercent MOT_GetSpeedPercent(const MOT_MotorDevice_t *motor);

/**
 * @brief Sets the speed of a motor, in the range of -100% (backward) to +100% (forward).
 * @param motor Motor handle.
 * @param percent Motor speed value, from -100 (full speed backward) to +100 (full speed forward).
 */
void MOT_SetSpeedPercent(MOT_MotorDevice_t *motor, MOT_SpeedPercent percent);

/**
 * @brief Updates the motor % speed based on actual PWM value.
 * @param motor Motor handle.
 * @param dir Current direction of motor.
 */
void MOT_UpdatePercent(MOT_MotorDevice_t *motor, MOT_Direction_t dir);

/**
 * @brief Sets the PWM value for the motor.
 * @param motor Motor handle
 * @param val New PWM value.
 */
void MOT_SetVal(MOT_MotorDevice_t *motor, uint16_t val);

/**
 * @brief Return the current PWM value of the motor.
 * @param motor Motor handle
 * @return Current PWM value.
 */
uint16_t MOT_GetVal(const MOT_MotorDevice_t *motor);

/**
 * @brief Change the direction of the motor
 * @param motor Motor handle
 * @param dir Direction to be used
 */
void MOT_SetDirection(MOT_MotorDevice_t *motor, MOT_Direction_t dir);

/**
 * @brief Returns the direction of the motor
 * @param motor Motor handle
 * @return Current direction of the motor
 */
MOT_Direction_t MOT_GetDirection(const MOT_MotorDevice_t *motor);

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
