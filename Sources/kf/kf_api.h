/***********************************************************************************************//**
 * @file		kf_api.h
 * @ingroup		kf
 * @brief 		API of the SWC @a Kalman Filter
 *
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	14.07.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 **************************************************************************************************/

#ifndef KF_API_H_
#define KF_API_H_

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
 * @param isLeft TRUE for left speed, FALSE for right speed.
 * @return Actual speed value
 */
EXTERNAL_ int32_t KF_Get_Speed(bool isLeft_);

EXTERNAL_ int32_t KF_GetPosition(bool isLeft_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !KF_API_H_ */
