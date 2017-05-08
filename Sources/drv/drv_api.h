/***********************************************************************************************//**
 * @file		drv_api.h
 * @ingroup		drv
 * @brief 		API of the SWC @a Drive
 *
 * This API provides a BSW-internal interface of the SWC @ref drv. It is supposed to be available
 * to all other Basic Software Components.
 *
 * @author 	(c) 2014 Erich Styger, erich.styger@hslu.ch, Hochschule Luzern
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.04.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef DRV_API_H_
#define DRV_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "rte_Types.h"
#include "Platform.h"


#ifdef MASTER_drv_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup drv
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @typedef DRV_Mode_t
 * @brief DRV_Mode_t is either inherited from DRV_Mode_t or defined as an enumeration
 *
 * For inheritance DRV_Mode_t must be defined within the Real-time environment (@ref rte) in
 * rte_Types.h. If it is not defined there it is defined in @ref drv as the enumeration
 * @ref DRV_Mode_e.
 */
#ifdef DRV_MODE_T
typedef DrvMode_t DRV_Mode_t;
#else
/**
 * @enum DRV_Mode_e
 * @brief Definition of available *drive modes*.
 */
typedef enum DRV_Mode_e {
  DRV_MODE_NONE, 		/**< No or invalid drive mode */
  DRV_MODE_STOP, 		/**< Stop mode for standstill control */
  DRV_MODE_SPEED,		/**< Speed mode for target velocity control */
  DRV_MODE_POS,  		/**< Position mode for target odometer control */
} DRV_Mode_t;
#endif

/**
 * @typedef DRV_Status_t
 * @brief The driving status information is hold in the struct @ref DRV_Status_s.
 *
 * @struct DRV_Status_s
 * @brief This data type defines the driving status information available.
 */
typedef struct DRV_Status_s {
	DRV_Mode_t mode;		/**< current [drive mode](@ref DRV_Mode_t) */
	struct {
		int32_t left;			/**< control target value for the left-hand side */
		int32_t right;			/**< control target value for the right-hand side */
	} speed;				/**< current controller target values in case of [DRV_MODE_SPEED](@ref DRV_Mode_t) implemented as anonymous struct */
	struct {
		int32_t left;
		int32_t right;
	} pos;					/**< current controller target values in case of [DRV_MODE_POS](@ref DRV_Mode_t) implemented as anonymous struct */
} DRV_Status_t;



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Sends a command to the queue to set a target [drive mode](@ref DRV_Mode_t)
 * @param mode target mode
 * @return Error code, ERR_OK if everything was fine\n
 * ERR_FAILED if sending to queues fails.
 */
EXTERNAL_ uint8_t DRV_SetMode(DRV_Mode_t mode);

/**
 * Returns the current [drive mode](@ref DRV_Mode_t).
 * @return current drive mode.
 */
EXTERNAL_ DRV_Mode_t DRV_GetMode(void);

/**
 * @brief Sends a command to the queue to set target speed values
 * @param left target speed value for the left-hand side in steps/sec
 * @param right target speed value for the right-hand side in steps/sec
 * @return Error code, ERR_OK if everything was fine\n
 * ERR_FAILED if sending to queues fails.
 */
EXTERNAL_ uint8_t DRV_SetSpeed(int32_t left, int32_t right);

/**
 * @brief Sends a command to the queue to set target position values
 * @param left target position value for the left-hand side in steps
 * @param right target position value for the right-hand side in steps
 * @return Error code, ERR_OK if everything was fine\n
 * ERR_FAILED if sending to queues fails.
 */
EXTERNAL_ uint8_t DRV_SetPos(int32_t left, int32_t right);

/**
 * @brief Returns TRUE if the robot is driving backwards.
 * @return TRUE/FALSE
 */
EXTERNAL_ bool DRV_IsDrivingBackward(void);

/**
 * @brief Returns TRUE if the robot is at standstill
 * @return TRUE/FALSE
 */
EXTERNAL_ bool DRV_IsStopped(void);

/**
 * @brief In [postion mode](@ref DRV_Mode_t) this function returns TRUE when the robot has just changed direction.
 * @return FALSE if the robot is driving faster than 50 steps/sec in position mode.\n TRUE otherwise.
 *
 */
EXTERNAL_ bool DRV_HasTurned(void);

/**
 * @brief Stops the engines
 * @param timeoutMs timout in milliseconds for operation
 * @return ERR_OK if stopped, ERR_BUSY for timeout condition.
 */
EXTERNAL_ uint8_t DRV_Stop(int32_t timeoutMs);

/**
 * @brief Returns the reference to the current [status information](@ref DRV_Status_t).
 * @return pointer to the status information
 */
EXTERNAL_ DRV_Status_t *DRV_GetCurStatus(void);

/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !DRV_API_H_ */
