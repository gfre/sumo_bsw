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
#include "ACon_Types.h"

#ifndef uchar_t
typedef unsigned char uchar_t;
#endif

#define KEY_PRESSED_NOTIFICATION_VALUE        (0x01u)
#define KEY_RELEASED_NOTIFICATION_VALUE       (0x02u)
#define KEY_PRESSED_LONG_NOTIFICATION_VALUE   (0x04u)
#define KEY_RELEASED_LONG_NOTIFICATION_VALUE  (0x08u)

#define BSW_API_VERSION      '1'
#define BSW_MAJOR_VERSION    '1'
#define BSW_MINOR_VERSION    '2'
#define BSW_YEAR             '2', '0', '1', '7'

#define BSW_VERSION_CHAR_ARRAY   BSW_API_VERSION, '.', BSW_MAJOR_VERSION, '.', BSW_MINOR_VERSION, '-', BSW_YEAR

#define BSW_VERSION_FORMAT_BYTE_COUNT		    (sizeof("1.2.3-2045"))


#endif /* PLATFORM_H_ */



