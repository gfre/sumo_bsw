/*******************************************************************************
 * @brief 	This is the interface entrance layer for students.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date	 	10.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#ifndef RTE_H
#define RTE_H

#include "Platform.h"
#include "CAU_Types.h"

#ifdef MASTER_RTE_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

typedef enum RTE_BuzTune_e {
	RTE_BUZ_TUNE_WELCOME = 0,
	RTE_BUZ_TUNE_BUTTON,
	RTE_BUZ_TUNE_BUTTON_LONG,
	RTE_BUZ_TUNE_NOF_TUNES
}RTE_BuzTune_t;

typedef enum RTE_DrvMode_e {
  RTE_DRV_MODE_NONE = 0,
  RTE_DRV_MODE_STOP,
  RTE_DRV_MODE_SPEED,
  RTE_DRV_MODE_POS,
  RTE_DRV_MODE_INVALID,
} RTE_DrvMode_t;

typedef void EvntCbFct_t(uint8 );

/**
 * @brief RTE interface to turn the right LED ON
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedRiOn();

/**
 * @brief RTE interface to turn the right LED OFF
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedRiOff();

/**
 * @brief RTE interface to toggle the state the right LED
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedRiNeg();

/**
 * @brief RTE interface to write the state of the right LED
 * @param state_ desired state of the LED
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedRiSt(uint8 state_);

/**
 * @brief RTE interface to read the state of the right LED
 * @param *state_ output: pointer to the LED state
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_LedRiSt(uint8 *state_);

/**
 * @brief RTE interface to turn the left LED ON
 * @param *state_ pointer to the LED state (call by reference)
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedLeOn();

/**
 * @brief RTE interface to turn the left LED OFF
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedLeOff();

/**
 * @brief RTE interface to toggle the state the left LED
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedLeNeg();

/**
 * @brief RTE interface to write the state of the left LED
 * @param state_ desired state of the LED
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedLeSt(uint8 state_);

/**
 * @brief RTE interface to read the state of the right LED
 * @param *state_ pointer to the LED state (call by reference)
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_LedLeSt(uint8 *state_);

/**
 * @brief RTE interface to read the state of the switch
 * @param *state_ pointer to the switch state (call by reference)
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_SwtSt(uint8 *state_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the switch is pressed shortly
 * @param *cbFct_ pointer to the callback function
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_SwtOnPrsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the switch is pressed for a longer time
 * @param *cbFct_ pointer to the callback function
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_SwtOnLngPrsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the switch is released after a short press
 * @param *cbFct_ pointer to the callback function
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_SwtOnRlsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the switch is released after a long press
 * @param *cbFct_ pointer to the callback function
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_SwtOnLngRlsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to get the pointer to a callback function
 * which is called when the switch is pressed shortly
 * @return pointer to the callback function
 */
EXTERNAL_ EvntCbFct_t *RTE_Get_SwtOnPrsdCbFct(void);

/**
 * @brief RTE interface to get the pointer to a callback function
 * which is called when the switch is pressed for a longer time
 * @return pointer to the callback function
 */
EXTERNAL_ EvntCbFct_t *RTE_Get_SwtOnLngPrsdCbFct(void);

/**
 * @brief RTE interface to get the pointer to a callback function
 * which is called when the switch is released after a short press
 * @return pointer to the callback function
 */
EXTERNAL_ EvntCbFct_t *RTE_Get_SwtOnRlsdCbFct(void);

/**
 * @brief RTE interface to get the pointer to a callback function
 * which is called when the switch is released after a long press
 * @return pointer to the callback function
 */
EXTERNAL_ EvntCbFct_t *RTE_Get_SwtOnLngRlsdCbFct(void);

/**
 * @brief RTE interface to play a buzzer tune
 * @param  tune_ enumeration to select a tune
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_BuzPlayTune(RTE_BuzTune_t tune_);

/**
 * @brief RTE interface to play a buzzer beep
 * @param  freqHz_ Frequncy of the Beep in Hertz
 *         durMs_  Duratoin of the Beep in milli seconds
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_BuzBeep(uint16 freqHz_, uint16 durMs_);

/**
 * @brief RTE interface to read the velocity of the left wheels
 * @param  *vel_ pointer to the current velocity in steps/sec (call by reference)
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_SpdoVelLe(uint16 *vel_);


/**
 * @brief RTE interface to read the velocity of the right wheels
 * @param  *vel_ pointer to the current velocity in steps/sec (call by reference)
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_SpdoVelRi(uint16 *vel_);

/**
 * @brief RTE interface to write the desired velocity in speed mode
 * @param  velLe_ desired velocity of left wheels in steps/sec
 * @param  velRi_ desired velocity of right wheels in steps/sec
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_DrvVel(int32 velLe_, int32 velRi_);

/**
 * @brief RTE interface to write the target postion in position mode
 * @param  posLe_ desired position of left wheels in steps/sec
 * @param  posRi_ desired position of right wheels in steps/sec
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_DrvPos(int32 posLe_, int32 posRi_);

/**
 * @brief RTE interface to command the desired driving control mode
 * @param  mode_ desired driving control mode (RTE_DrvMode_t)
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_DrvMode(RTE_DrvMode_t mode_);

/**
 * @brief RTE interface to read the current driving control mode
 * @param  *mode_ pointer to the current driving control mode (RTE_DrvMode_t)
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_DrvMode(RTE_DrvMode_t *mode_);

/**
 * @brief RTE interface to read if the sumo is driving backwards
 * @param  *isDrvgBkwd_ pointer to a flag
 *                        TRUE  - driving backward,
 *                        FALSE - driving forward
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_DrvIsDrvgBkwd(uint8 *isDrvgBkwd_);

/**
 * @brief RTE interface to read if the sumo has stopped
 * @param  *hasStopped_ pointer to a flag
 *                        TRUE  - has stopped,
 *                        FALSE - still driving
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_DrvHasStpd(uint8 *hasStpd_);

/**
 * @brief RTE interface to read if the sumo has just reversed
 * @param  *hasStopped_ pointer to a flag
 *                        TRUE  - has just reversed,
 *                        FALSE - has not just reversed
 * @return Error code, RTN_OK if everything was fine,
 *                     RTN_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_DrvHasRvsd(uint8 *hasRvsd_);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* RTE_H */
