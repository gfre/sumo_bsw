/***********************************************************************************************//**
 * @file		kf_api.h
 * @ingroup		kf
 * @brief 		API of the SWC @a Kalman filter
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author 	S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	14.07.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef KF_API_H_
#define KF_API_H_

/**
 * @addtogroup kf
 * @{
 */
/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"



#ifdef MASTER_KF_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup kf
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
 /**
 * @brief Returns the previously calculated speed of the motor.
 * @param[in,out] pVal_ speed in steps/sec
 * @param[in] idx_ current kf id
 * @return ERR_PARAM_ADDRESS if any pointer is NULL<br>
 * 		   ERR_PARAM_VALUE if idx_ exceeds total number of KFs
 */
EXTERNAL_ StdRtn_t KF_Read_i16EstdVal(int16_t *pVal_, const uint8_t idx_);


/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !KF_API_H_ */
