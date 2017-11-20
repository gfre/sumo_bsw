/***********************************************************************************************//**
 * @file		kf.h
 * @ingroup		kf Kalman Filter
 * @brief 		Interface of the SWC @ref kf for the initialization- and runtime calls
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author 	S. Helling,  stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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
/**
 * String identification of the SWC @ref kf
 */
#define KF_SWC_STRING	("kalman filter")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION Declarations << ================================*/
/**
 * @brief Initialisation function of the software component @ref kf
 *
 * This function initializes the configured objects from kf_cfg.c
 */
EXTERNAL_ void KF_Init(void);




/**
 * @brief Main function of the software component @ref kf
 *
 * This function runs the implementation of the kalman filter
 */
EXTERNAL_ void KF_Main(void);



/**
 * @brief De-Initialisation function of the software component @ref kf
 *
 * This function de-initializes the configured objects from kf_cfg.c
 */
EXTERNAL_ void KF_Deinit(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* KF_H_ */
