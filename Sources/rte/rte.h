/*
 * rte.h
 *
 *  Created on: 10.01.2017
 *      Author: gefr
 */

#ifndef RTE_H
#define RTE_H

#include "Platform.h"

#ifdef MASTER_RTE_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

EXTERNAL_ StdRetType RTE_Write_LedRiOn();

EXTERNAL_ StdRetType RTE_Write_LedRiOff();

EXTERNAL_ StdRetType RTE_Write_LedRiNeg();

EXTERNAL_ StdRetType RTE_Write_LedRiSt(uint8_t state);

EXTERNAL_ StdRetType RTE_Read_LedRiSt(uint8_t *state);

EXTERNAL_ StdRetType RTE_Write_LedLeOn();

EXTERNAL_ StdRetType RTE_Write_LedLeOff();

EXTERNAL_ StdRetType RTE_Write_LedLeNeg();

EXTERNAL_ StdRetType RTE_Write_LedLeSt(uint8_t state);

EXTERNAL_ StdRetType RTE_Read_LedLeSt(uint8_t *state);




#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !RTE_H */
