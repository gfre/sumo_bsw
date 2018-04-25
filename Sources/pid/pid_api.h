/***********************************************************************************************//**
 * @file		pid_api.h
 * @ingroup		pid
 * @brief 		API of the SWC @a PID
 *
 * This API provides a BSW-internal interface of the SWC @ref pid. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author  S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	27.04.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef PID_API_H_
#define PID_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"
#include "Platform.h"
#include "nvm_api.h"

#ifdef MASTER_pid_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup pid
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the SWC @ref pid
 */
#define PID_SWC_STRING ("PID controller")



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 *
 */
typedef struct PID_Gain_s
{
	uint16_t kP_scld;
	uint16_t kI_scld;
	uint16_t kD_scld;
	uint16_t nScale; // TODO change to uint8_t
	uint32_t intSatVal;
}PID_Gain_t;

/**
 *
 */
typedef enum PID_Sat_e
{
	 PID_NEG_SAT = -1
	,PID_NO_SAT = 0
	,PID_POS_SAT = 1
}PID_Sat_t;

/**
 *
 */
typedef struct PID_Data_s
{
	PID_Sat_t sat;
	int32_t	intVal;
	int32_t	prevErr;
}PID_Data_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Calculates control value from error between setVal_ and actVal_
 * @param setVal_ The desired value
 * @param actVal_ The current value
 * @param idx_    ID that corresponds to an item from the @ref pid_cfg.c file
 * @param ctrlVal_ The output control value
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_INDEX if  idx_ doesn't exist in @ref PID_ItmTbl_t,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t PID(int32_t setVal_, int32_t actVal_, uint8_t idx_, int32_t* ctrlVal_);

/**
 * @brief Calculates control value from error between setVal_ and actVal_ similar to
 *        @ref PID but can be called from outside of PID scope (e.g. from @ref tl)
 * @param setVal_ The desired value
 * @param actVal_ The current value
 * @param gain_   Contains the proportional, integral and differential gain, aswell
 *                as saturation information and scaling information
 * @param data_   Contains previous error, integral value, and saturation type
 * @param ctrlVal_ The output control value
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t PIDext(int32_t setVal_, int32_t actVal_, const PID_Gain_t *gain_, PID_Data_t *data_, int32_t* ctrlVal_);

/**
 * @brief Resets a PID item by resetting runtime data
 * @param idx_ ID of PID item in @ref PID_ItmTbl_t that is to be resetted
 * @return Error code,  ERR_OK if everything was fine,
 *                      ERR_PARAM_IDX if idx_ doesn't exist in @ref PID_ItmTbl_t,
 *                      ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t PID_Reset(uint8_t idx_);
/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !PID_API_H_ */
