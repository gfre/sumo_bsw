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
#define NOF_STATES 2
#define NOF_MSRD_STATES 2
#define NOF_INPUTS 0


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef StdRtn_t ReadValFct_t(int32_t*);
typedef struct KF_MtxPrm_s
{
		MTX_t A[NOF_STATES][NOF_STATES];
#if NOF_INPUTS > 0
		MTX_t B[NOF_STATES][NOF_INPUTS];
#endif
		MTX_t C[NOF_MSRD_STATES][NOF_STATES];
		MTX_t R[NOF_MSRD_STATES][NOF_MSRD_STATES];
		MTX_t Q[NOF_STATES][NOF_STATES];
		MTX_t x0[NOF_STATES][1];
		MTX_t P0[NOF_STATES][NOF_STATES];
}KF_MtxPrm_t;

typedef struct KF_Itm_s
{
	KF_MtxPrm_t  *MatrixConfig;
	ReadValFct_t *pGetMsrmntValFct;
	ReadValFct_t *pGetInputValFct;
}KF_Itm_t;

typedef struct KF_Cfg_s
{
	KF_Itm_t *itmTbl;
	uint8_t NumOfItms;
}KF_Cfg_t;

KF_MtxPrm_t mtxCfg = {
		{{1000, 5}, {0, 1000}}, //A
		{{1,    0}, {0, 1}}, 	//C
		{{3,    0}, {0, 20000}},//R
		{{10,   0}, {0, 2500}}, //Q
		{{0},       {0}},		//x0
		{{100,  0}, {0, 100}}	//P0
};

KF_Itm_t kfItmTbl[] =
{
		{&mtxCfg, TACHO_Read_CurLftPos, NULL},
		{&mtxCfg, TACHO_Read_CurRghtPos, NULL},
};

KF_Cfg_t kfCfg =
{
	kfItmTbl,
	sizeof(kfItmTbl)/sizeof(kfItmTbl[0]),
};



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
/* static StdRtnType STUD_CustomFct(const studType_t *input_, studType_t *output_); */



/*=================================== >> GLOBAL VARIABLES << =====================================*/
/* static studType_t studArray[STUD_MACRO] = {0u,TRUE,FALSE}; */



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
/* static StdRtnType STUD_CustomFct(const studType_t *input_, studType_t *output_)
 * {
 * 		Write your code here!
 * }
 */



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void DAPP_Init(void)
{
}




void DAPP_Main(void)
{
MTX_t m1[2][3] = {{1,3,4},{1,2,6}};
MTX_t m2[3][3] = {{1,3,4},{2,3,4},{1,2,0}};
MTX_t m3[2][3] = {0};
MTX_t m4[3][3] = {0};
MTX_t v1[1][3] = {1, 3, 4};
MTX_t v15[1][3] = {0};
MTX_t v2[3][1] = {4, 3 ,0};
MTX_t v25[3][1] = {0};
MTX_t fac[3][3] = {{2,0,0},{0,2,0},{0,0,2}};
MTX_t fac2[1][1] = {2};

MTX_Add(2, 3, m1, 2, 3, m1, m3);
MTX_Mult(2, 3, m1, 3, 3, m2, m3);
MTX_Trns(3, 3, m2, 0, 0, NULL, m4);
MTX_Mult(3 ,3, fac,  3, 3, m2, m4); //factor * matrix
MTX_Mult(1, 1, fac2, 1, 3, v1, v15); //factor * rowvector TODO
MTX_Mult(3, 1, v2, 1, 1, fac2, v25); //factor * colvec

int32_t c = mtxCfg.A[0][0];
c = mtxCfg.C[1][1];
MTX_t* d = mtxCfg.C;



}

#ifdef MASTER_STUD_C_
#undef MASTER_STUD_C_
#endif
