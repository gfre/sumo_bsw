/*******************************************************************************
 * @brief 	This provides basic macros for the CAUsumo adaption.
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel
 * @date 	09.01.2017
 *
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 *
 * ==============================================================================
 */

#ifndef CAU_TYPES_H
#define CAU_TYPES_H
#include "PE_Types.h"
#include <stddef.h> /* for size_t */

/**
* Type Definition for Standard Return Type
*/
typedef uint8 StdRtnType;

/**
    Standard macro for pressed buttons
*/
#define BTN_PRESSED (1u)

/**
    Standard macro for not pressed buttons
*/
#define BTN_NPRESSED (0u)

/**
    Standard macro for positive return code
*/
#define RTN_OK (0u)

/**
    Standard macro for negative return code
*/
#define RTN_INVALID (1u)


typedef enum SigStatus_tag_
{
	SIG_OK=0           /**< Signal status is ok. */
   ,SIG_INVALID=1      /**< Signal status is invalid. */
   ,SIG_ERROR=2        /**< Signal value shows error. */
   ,SIG_NA=3           /**< Signal value shows not available. */
   ,SIG_OUT_OF_RANGE=4 /**< Signal value is out of specified signal range. */
}SigStatus;


/**
* Type definition for uint8 with signal status
*/

typedef struct UInt8WithStatus_s
{
    uint8 value;      /**< Signal value [0..2^8-1]. */
    SigStatus status; /**< Signal status. */
}UInt8WithStatus;

/**
* Type definition for sint8 with signal status
*/

typedef struct SInt8WithStatus_s
{
    int8 value;       /**< Signal value [-2^7..2^7-1]. */
    SigStatus status; /**< Signal status. */
}SInt8WithStatus;

/**
* Type definition for uint16 with signal status
*/

typedef struct UInt16WithStatus_s
{
    uint16 value;     /**< Signal value [0..2^16-1]. */
    SigStatus status; /**< Signal status. */
}UInt16WithStatus;

/**
* Type definition for sint16 with signal status
*/

typedef struct SInt16WithStatus_s
{
    int16 value;      /**< Signal value [-2^15..2^15-1]. */
    SigStatus status; /**< Signal status. */
}SInt16WithStatus;

/**
* Type Definition for uint32 with signal status
*/

typedef struct UInt32WithStatus_s
{
    uint32 value;     /**< Signal value [0..2^32-1]. */
    SigStatus status; /**< Signal status. */
}UInt32WithStatus;

/**
* Type definition for sint32 with signal status
*/

typedef struct SInt32WithStatus_s
{
    int32 value;      /**< Signal value [-2^31..2^31-1]. */
    SigStatus status; /**< Signal status. */
}SInt32WithStatus;


#endif  /* !CAU_TYPES_H */
