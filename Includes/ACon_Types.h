/***********************************************************************************************//**
 * @file		ACon_Types.h
 * @ingroup		incl
 * @brief 		Basic macros and data type definitions for C-programming projects
 *
 * This header file collects and provides common preprocessor macros and data type definitions for
 * C-programming projects within the department 'Chair of Automatic Control' at University Kiel
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.01.2017
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef ACON_TYPES_H
#define ACON_TYPES_H

/*======================================= >> #INCLUDES << ========================================*/
#include <stdint.h>



/*======================================= >> #DEFINES << =========================================*/
#ifndef   MIN
  #define MIN(a, b)         (((a) < (b)) ? (a) : (b))
#endif

#ifndef   MAX
  #define MAX(a, b)         (((a) > (b)) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(a)              (((a) > (0)) ? (a) : ((-1)*(a)))
#endif

#ifndef SIGN
#define SIGN(a)             (((a) < (0)) ? (-1) : (1))
#endif

#define FOUR_PI             (12.56637061436)
#define THREE_PI            ( 9.42477796077)
#define TWO_PI              ( 6.28318530718)
#define THREE_HALF_PI       ( 4.71238898038)
#define PI                  ( 3.14159265359)
#define THREE_QUARTERS_PI   ( 2.35619449019)
#define TWO_THIRD_PI        ( 2.09439510239)
#define ONE_HALF_PI         ( 1.57079632679)
#define ONE_THIRD_PI        ( 1.04719755120)
#define ONE_QUARTER_PI      ( 0.78539816340)


/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
* Type Definition for Standard Return Type
*/
typedef uint8_t StdRtn_t;

/**
    Standard macro for pressed buttons
*/
#define BTN_PRESSED (1u)

/**
    Standard macro for not pressed buttons
*/
#define BTN_NPRESSED (0u)

/**
 * @typedef uchar_t
 * @brief Type definition of an unsigned character, e.g. for string representations
 */
#ifndef uchar_t
typedef unsigned char uchar_t;
#endif

/**
 * @typedef SigStatus_t
 * @brief
 *
 * @enum SigStatus_e
 * @brief
 */
typedef enum SigStatus_e
{
    SIG_OK=0           /**< Signal status is ok. */
   ,SIG_INVALID=1      /**< Signal status is invalid. */
   ,SIG_ERROR=2        /**< Signal value shows error. */
   ,SIG_NA=3           /**< Signal value shows not available. */
   ,SIG_OUT_OF_RANGE=4 /**< Signal value is out of specified signal range. */
}SigStatus_t;


/**
* Type definition for uint8 with signal status
*/

typedef struct UInt8WithStatus_s
{
    uint8_t value;      /**< Signal value [0..2^8-1]. */
    SigStatus_t status; /**< Signal status. */
}UInt8WithStatus_t;

/**
* Type definition for sint8 with signal status
*/

typedef struct SInt8WithStatus_s
{
    int8_t value;       /**< Signal value [-2^7..2^7-1]. */
    SigStatus_t status; /**< Signal status. */
}SInt8WithStatus_t;

/**
* Type definition for uint16 with signal status
*/

typedef struct UInt16WithStatus_s
{
    uint16_t value;     /**< Signal value [0..2^16-1]. */
    SigStatus_t status; /**< Signal status. */
}UInt16WithStatus_t;

/**
* Type definition for sint16 with signal status
*/

typedef struct SInt16WithStatus_s
{
    int16_t value;      /**< Signal value [-2^15..2^15-1]. */
    SigStatus_t status; /**< Signal status. */
}SInt16WithStatus_t;

/**
* Type Definition for uint32 with signal status
*/

typedef struct UInt32WithStatus_s
{
    uint32_t value;     /**< Signal value [0..2^32-1]. */
    SigStatus_t status; /**< Signal status. */
}UInt32WithStatus_t;

/**
* Type definition for sint32 with signal status
*/

typedef struct SInt32WithStatus_s
{
    int32_t value;      /**< Signal value [-2^31..2^31-1]. */
    SigStatus_t status; /**< Signal status. */
}SInt32WithStatus_t;


#endif  /* !ACON_TYPES_H */
