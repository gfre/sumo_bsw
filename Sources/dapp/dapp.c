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


/*======================================= >> #DEFINES << =========================================*/
/* #define STUD_MACRO (3) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/



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
MTX_Add(2, 3, m1, 3, 3, m2, m3);
}

#ifdef MASTER_STUD_C_
#undef MASTER_STUD_C_
#endif
