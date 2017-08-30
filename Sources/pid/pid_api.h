/***********************************************************************************************//**
 * @file		pid_api.h
 * @ingroup		pid
 * @brief 		API of the SWC @a PID
 *
 * This API provides a BSW-internal interface of the SWC @ref pid. It is supposed to be
 * available to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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
	int32_t	prevErr;
	int32_t	intVal;
}PID_Data_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 *
 * @param gain_
 * @param setVal_
 * @param actVal_
 * @param rtData_
 * @param ctrlVal_
 * @return
 */
EXTERNAL_ StdRtn_t PID(int32_t setVal_, int32_t actVal_, uint8_t idx_, int32_t* ctrlVal_);

/**
 *
 * @param setVal_
 * @param actVal_
 * @param gain_
 * @param data_
 * @param ctrlVal_
 * @return
 */
EXTERNAL_ StdRtn_t PIDext(int32_t setVal_, int32_t actVal_, const PID_Gain_t *gain_, PID_Data_t *data_, int32_t* ctrlVal_);

/**
 *
 * @param idx_
 * @return
 */
EXTERNAL_ StdRtn_t PID_Reset(uint8_t idx_);
/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !PID_API_H_ */
