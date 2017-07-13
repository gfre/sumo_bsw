/***********************************************************************************************//**
 * @file		kf_cfg.h
 * @ingroup		kf Kalman Filter
 * @brief 		This header file contains the type definitions for matrices and column and row vectors
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
#include "Platform.h"
#include "Acon_Types.h"


#ifdef MASTER_KF_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


/*======================================= >> #DEFINES << =========================================*/
#define KF_SYS_DIMENSION (0x02u)
#define KF_SCALE_KALMANGAIN (100u)
#define KF_SCALE_ERROR (50u)
#define KF_SCALE_A (1000u)
#define KF_SCALE_X (100u)
#define KF_MAX_POS_VAL (2000000000u)


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct KF_I32ColVec_s {
	int32_t aRow[KF_SYS_DIMENSION];
}KF_I32ColVec_t;

typedef struct KF_I32RowVec_s {
	int32_t aCol[KF_SYS_DIMENSION];
}KF_I32RowVec_t;

typedef struct KF_I32Matrix_s {
	KF_I32RowVec_t aRow[KF_SYS_DIMENSION];
}KF_I32Mat_t;

typedef struct{
	KF_I32Mat_t* 	SystemMatrix;
	KF_I32RowVec_t* MeasurementVectorTransposed;
	KF_I32Mat_t* 	IdentityMatrix;
	int32_t* 	 	MeasurementNoiseCov;
	KF_I32Mat_t* 	ProcessNoiseCov;
	KF_I32Mat_t* 	InitialErrorInEstimate;
	KF_I32ColVec_t* StateInitialEstimate;
}KF_Cfg_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ const KF_Cfg_t *GetKFCfg(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* KF_CFG_H */
