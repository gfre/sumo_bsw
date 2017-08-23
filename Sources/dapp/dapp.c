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


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct TestMatCfg_s
{
	MTX_t *A;
	MTX_t *B;
}TestMatCfg_t;
/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/



/*=================================== >> GLOBAL VARIABLES << =====================================*/
int32_t A[2][3] = {{1,3,4},{1,2,6}};
int32_t B[3][5] = {{1,3,3,4,5},{2,3,1,4,6},{1,0,1,3,2}};

MTX_t Mat_A = {A[0], 2, 3};
MTX_t Mat_B = {B[0], 3, 5};

TestMatCfg_t cfg = {&Mat_A, &Mat_B};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void DAPP_Init(void)
{


}


void DAPP_Main(void)
{
	int32_t temp[2][5] = {0u};
	MTX_t tempMat= {temp[0], 2, 5};
	MTX_Mult(*cfg.A, *cfg.B, &tempMat);
}

#ifdef MASTER_STUD_C_
#undef MASTER_STUD_C_
#endif
