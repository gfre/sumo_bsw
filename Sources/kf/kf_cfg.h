/***********************************************************************************************//**
 * @file		kf_cfg.h
 * @ingroup		kf Kalman Filter
 * @brief 		This header file contains the type definitions for matrices and column and row vectors
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
#define KF_SCALE_KALMANGAIN (1000u)
#define KF_SCALE_ERROR (50u)
#define KF_SCALE_A (1000u)
#define KF_SCALE_X (100u)
#define KF_SCALE_DET (1000u)  //needs to be at least one decade grater than SCALE_ERROR
#define KF_MAX_POS_VAL (2000000000u)
#define KF_USE_MEASUREMENT_MATRIX (0u) //set to 1 if using more than 1 measurement:this turns c from a vector to a matrix and makes calculation of inverse necessary thus changing the entire algorithm


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

typedef struct KF_I32RowVecLowDim_s { //necessary for determinant using cramers rule
	int32_t aCol[KF_SYS_DIMENSION-1];
}KF_I32RowVecLowDim_t;

typedef struct KF_I32MatrixLowDim_s {
	KF_I32RowVecLowDim_t aRow[KF_SYS_DIMENSION-1];
}KF_I32MatLowDim_t;

typedef struct{
	KF_I32Mat_t* 	I;
	KF_I32Mat_t* 	A;
	KF_I32Mat_t* 	B;
	KF_I32RowVec_t* cT;
	KF_I32Mat_t* 	C;
	KF_I32ColVec_t* x0;
	int32_t* 	 	r;
	KF_I32Mat_t* 	R;
	KF_I32Mat_t* 	Q;
	KF_I32Mat_t* 	P0;

}KF_Cfg_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ const KF_Cfg_t *GetKFCfg(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* KF_CFG_H */
