/***********************************************************************************************//**
 * @file		kf.h
 * @ingroup		kf Kalman Filter
 * @brief 		This header file contains the type definitions for matrices and column and row vectors
 *
 *
 * @author 	S. Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	23.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#define KF_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"
#include "Acon_Types.h"

#ifdef MASTER_KF_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#define KF_SYS_DIMENSION	(0x02u)
#define KF_SWC_STRING		("kalman filter")
#define KF_SCALE_KALMANGAIN 1000
#define KF_SCALE_ERROR 10
#define KF_SCALE_A 1000
#define KF_SCALE_X 10


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

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void KF_Init(void);
void KF_Main(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif /* KF_H_ */
