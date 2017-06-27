/***********************************************************************************************//**
 * @file		kf_cfg.h
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	<I>. <Surname>, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	26.06.2017
 *
 * @copyright @<LGPLx_x>
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


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct{
	KF_Matrix_t* SystemMatrix;
	KF_Matrix_t* MeasurementMatrix;
	KF_Matrix_t* IdentityMatrix;
	KF_Matrix_t* MeasurementNoiseCovarianceMatrix;
	KF_Matrix_t* ProcessNoiseCovarianceMatrix;
	KF_Matrix_t* InitialErrorCovarianceMatrix;
	KF_Vector_t* InitialValues;
	KF_Vector_t* InitialEstimate;
}KF_Cfg_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ const KF_Cfg_t *GetKFCfg(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* KF_CFG_H */
