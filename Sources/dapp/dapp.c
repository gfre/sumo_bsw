/***********************************************************************************************//**
 * @file		dapp.c
 * @ingroup		dapp
 * @brief 		Simple framework for demo application software.
 *
 * This module implements a simple framework for custom demo application, e.g., for demonstration,
 * testing, debugging or education purposes at Univeristy Kiel.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	09.01.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_DAPP_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "dapp.h"
#include "mtx_api.h"
#include "tacho_api.h"

/*======================================= >> #DEFINES << =========================================*/
#define GET_NUM_ROWS(mtx_) (sizeof(mtx_)/sizeof(mtx_[0]))
#define GET_NUM_COLS(mtx_) (sizeof(mtx_[0])/sizeof(mtx_[0][0]))

/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct TestMatCfg_s
{
	MTX_t *A;
	MTX_t *B;
	MTX_t *C;
	MTX_t *x;
}TestMatCfg_t;
/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
int32_t A[2][3] = {{1,3,4},{1,2,6}};
int32_t B[3][5] = {{1,3,3,4,5},{2,3,1,4,6},{1,0,1,3,2}};
int32_t C[2][3] = {{1,3,4},{1,2,6}};
int32_t x[3][1] = {{4},{5},{6}};
int32_t rowvec[1][3] = {{1,3,4}};

MTX_t Mat_A = {A[0], sizeof(A)/sizeof(A[0]), sizeof(A[0])/sizeof(A[0][0])};
MTX_t Mat_B = {B[0], sizeof(B)/sizeof(B[0]), sizeof(B[0])/sizeof(int32_t)};
MTX_t Mat_C = {C[0], GET_NUM_ROWS(C), GET_NUM_COLS(C)};
MTX_t Vec_x = {x[0], sizeof(x)/sizeof(x[0]), sizeof(x[0])/sizeof(int32_t)};
MTX_t RowVec = {rowvec[0], sizeof(rowvec)/sizeof(rowvec[0]), sizeof(rowvec[0])/sizeof(int32_t)};

TestMatCfg_t cfg = {&Mat_A, &Mat_B, &Mat_C, &RowVec};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void DAPP_Init(void)
{


}


void DAPP_Main(void)
{
	int32_t temp[2][5] = {0u};
	MTX_t tempMat= {temp[0], 2, 5};
	MTX_Mult(cfg.A, cfg.B, &tempMat);

	int32_t temp2[2][1] = {0u};
	MTX_t tempVec = {temp2[0],2,1};
	MTX_Mult(cfg.A, cfg.x, &tempVec);

	int32_t tempRowVec[1][5] = {0u};
	MTX_t tempRowVec1 = {tempRowVec[0],1,5};
	MTX_Mult(cfg.x, cfg.B, &tempRowVec1);

	int32_t temp23Mat[2][3] = {0u};
	MTX_t temp23_Mat  = {temp23Mat[0], 2,3};
	MTX_Add(cfg.C, cfg.A, &temp23_Mat);

}

#ifdef MASTER_STUD_C_
#undef MASTER_STUD_C_
#endif
