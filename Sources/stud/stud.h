/*******************************************************************************
 * @brief 	Simple framework for custom application software.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date 	14.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This module implements a simple framework for custom applications software
 * for education at Univeristy Kiel.
 *
 * ==============================================================================
 */

#ifndef STUD_H_
#define STUD_H_

#include "rte.h"

#ifdef MASTER_STUD_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif




/*!
 * @brief Init function of the software component 'stud'
 * Initializes custom application software
 */
EXTERNAL_ void STUD_Init(void);


/*!
 * @brief Main function of the software component 'stud'
 * Runs custom application software
 */
EXTERNAL_ void STUD_Main(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !STUD_H_ */
