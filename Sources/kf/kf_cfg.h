/***********************************************************************************************//**
 * @file		kf_cfg.h
 * @ingroup		kf Kalman Filter
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	26.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef KF_CFG_H
#define KF_CFG_H

/*======================================= >> #INCLUDES << ========================================*/

#ifdef MASTER_KF_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#define DIVIDER (1000)

/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct{
	KF_I32Mat_t* 	SystemMatrix;
	KF_I32RowVec_t* MeasurementVector;
	KF_I32Mat_t* 	IdentityMatrix;
	int32_t* 	 	MeasurementNoiseCovariance;
	KF_I32Mat_t* 	ProcessNoiseCovarianceMatrix;
	KF_I32Mat_t* 	InitialErrorCovarianceMatrix;
	KF_I32ColVec_t* InitialEstimate;
}KF_Cfg_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ const KF_Cfg_t *GetKFCfg(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* KF_CFG_H */
