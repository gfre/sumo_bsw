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
#include "Buzzer.h"

#ifdef MASTER_RTE_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

typedef void EvntCbFct_t(uint8 );

/**
 * @brief RTE interface to turn the right LED ON
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedRiOn();

/**
 * @brief RTE interface to turn the right LED OFF
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedRiOff();

/**
 * @brief RTE interface to toggle the state the right LED
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedRiNeg();

/**
 * @brief RTE interface to write the state of the right LED
 * @param state_ desired state of the LED
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedRiSt(uint8 state_);

/**
 * @brief RTE interface to read the state of the right LED
 * @param *state_ output: pointer to the LED state
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_LedRiSt(uint8 *state_);

/**
 * @brief RTE interface to turn the left LED ON
 * @param *state_ pointer to the LED state (call by reference)
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedLeOn();

/**
 * @brief RTE interface to turn the left LED OFF
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedLeOff();

/**
 * @brief RTE interface to toggle the state the left LED
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedLeNeg();

/**
 * @brief RTE interface to write the state of the left LED
 * @param state_ desired state of the LED
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_LedLeSt(uint8 state_);

/**
 * @brief RTE interface to read the state of the right LED
 * @param *state_ pointer to the LED state (call by reference)
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_LedLeSt(uint8 *state_);

/**
 * @brief RTE interface to read the state of the switch
 * @param *state_ pointer to the switch state (call by reference)
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_SwtSt(uint8 *state_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the switch is pressed shortly
 * @param *cbFct_ pointer to the callback function
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_SwtOnPrsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the switch is pressed for a longer time
 * @param *cbFct_ pointer to the callback function
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_SwtOnLngPrsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the switch is released after a short press
 * @param *cbFct_ pointer to the callback function
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Write_SwtOnRlsdCbFct(const EvntCbFct_t *cbFct_);

/**
 * @brief RTE interface to set the pointer to a callback function
 * which is called when the switch is released after a long press
 * @param *cbFct_ pointer to the callback function
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
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
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Play_BuzTune(BUZ_Tunes tune_);

/**
 * @brief RTE interface to play a buzzer beep
 * @param  freqHz_ Frequncy of the Beep in Hertz
 *         durMs_  Duratoin of the Beep in milli seconds
 * @return Error code, RET_OK if everything was fine,
 *                     RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Play_BuzBeep(uint16 freqHz_, uint16 durMs_);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* RTE_H */
