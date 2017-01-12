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

EXTERNAL_ StdRtnType RTE_Write_LedRiOn();

EXTERNAL_ StdRtnType RTE_Write_LedRiOff();

EXTERNAL_ StdRtnType RTE_Write_LedRiNeg();

EXTERNAL_ StdRtnType RTE_Write_LedRiSt(uint8 state_);

EXTERNAL_ StdRtnType RTE_Read_LedRiSt(uint8 *state_);

EXTERNAL_ StdRtnType RTE_Write_LedLeOn();

EXTERNAL_ StdRtnType RTE_Write_LedLeOff();

EXTERNAL_ StdRtnType RTE_Write_LedLeNeg();

EXTERNAL_ StdRtnType RTE_Write_LedLeSt(uint8 state_);

EXTERNAL_ StdRtnType RTE_Read_LedLeSt(uint8 *state_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* RTE_H */
