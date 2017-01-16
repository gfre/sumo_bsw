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

#ifdef MASTER_RTE_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

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
 * @return Error code, RET_OK if everything was fine, RET_INVALID otherwise
 */
EXTERNAL_ StdRtnType RTE_Read_LedLeSt(uint8 *state_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* RTE_H */
