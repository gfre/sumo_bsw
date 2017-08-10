/***********************************************************************************************//**
 * @file		kf.h
 * @ingroup		kf Kalman Filter
 * @brief 		This header file contains the init and main function declarations as part of the task
 * 				component drive
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	23.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef KF_H_
#define KF_H_

/*======================================= >> #INCLUDES << ========================================*/



#ifdef MASTER_KF_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#define KF_FILTER_STRING	("kalman filter")

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void KF_Init(void);
void KF_Main(void);
void KF_Deinit(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* KF_H_ */
