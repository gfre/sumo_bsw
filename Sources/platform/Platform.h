/*******************************************************************************
 * @brief 	This provides basic includes and constants/macros for CAUsumo.
 *
 * @author 	Henning Weisbarth, hewe@tf.uni-kiel.de, CAU Kiel
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date 		03.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

#include "Cpu.h" /* for PEcfg_FreeMASTER */
#include <stddef.h> /* for size_t */


#define KEY_PRESSED_NOTIFICATION_VALUE      (0x01u)
#define KEY_PRESSED_LONG_NOTIFICATION_VALUE (0x02u)

#define SW_API_VERSION      '1'
#define SW_MAJOR_VERSION    '0'
#define SW_MINOR_VERSION    '0'
#define SW_YEAR             "2017"

#define SW_VERSION_CHAR_ARRAY   SW_API_VERSION, '.', SW_MAJOR_VERSION, '.', SW_MINOR_VERSION,'-'

#endif /* PLATFORM_H_ */
