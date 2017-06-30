/***********************************************************************************************//**
 * @file		kf.h
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	<I>. <Surname>, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	23.06.2017
 *
 * @copyright @<LGPLx_x>
 *
 ***************************************************************************************************/

#define MASTER_KF_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"
#include "Acon_Types.h"

/*======================================= >> #DEFINES << =========================================*/
#define KF_SYS_DIMENSION	(0x02u)



/*=================================== >> TYPE DEFINITIONS << =====================================*/
//typedef struct KF_I16Vec_s {
//	int16_t aRow[KF_SYS_DIMENSION];
//}KF_I16Vec_t;
//
//typedef struct KF_I16VecCol_s {
//	int16_t aCol[KF_SYS_DIMENSION];
//}KF_I16VecCol_t;
//
//typedef struct KF_I16Matrix_s {
//	KF_I16VecCol_t aRow[KF_SYS_DIMENSION];
////	uint8_t dim;
//}KF_I16Mat_t;

typedef struct KF_I32ColVec_s {
	int32_t aRow[KF_SYS_DIMENSION];
}KF_I32ColVec_t;

typedef struct KF_I32RowVec_s {
	int32_t aCol[KF_SYS_DIMENSION];
}KF_I32RowVec_t;

typedef struct KF_I32Matrix_s {
	KF_I32RowVec_t aRow[KF_SYS_DIMENSION];
//	uint8_t dim;
}KF_I32Mat_t;

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void KF_Init(void);
void KF_Main(void);


#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
