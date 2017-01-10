#ifndef CAU_TYPES_H
#define CAU_TYPES_H
#include "PE_Types.h"
#include <stddef.h> /* for size_t */

/**
* Type Definition for Standard Return Type
*/
typedef uint8_t StdRetType;

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
#define RET_OK (0u)

/**
    Standard macro for negative return code
*/
#define RET_INVALID (1u)


typedef enum SigStatusType_
{
	SIG_OK=0           /**< Signal status is ok. */
   ,SIG_INVALID=1      /**< Signal status is invalid. */
   ,SIG_ERROR=2        /**< Signal value shows error. */
   ,SIG_NA=3           /**< Signal value shows not available. */
   ,SIG_OUT_OF_RANGE=4 /**< Signal value is out of specified signal range. */
}SigStatusType;


#endif  /* !CAU_TYPES_H */
