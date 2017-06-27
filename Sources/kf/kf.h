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
#include "PE_Types.h"

/*======================================= >> #DEFINES << =========================================*/
#define DIMENSION	(0x02u)
#define INIT_MATRIX_DIM_2 {{0, 0}, {0, 0},}
#define INIT_MATRIX_DIM_3 {{0, 0, 0}, {0, 0, 0}, {0, 0, 0},}
#define INIT_VECTOR_DIM_2 {{0}, {0},}
#define INIT_VECTOR_DIM_3 {{0}, {0}, {0},}



/*=================================== >> TYPE DEFINITIONS << =====================================*/
//typedef struct {
//	double column[1];
//}Vector;
//
//typedef struct {
//	double column[2];
//}Matrix;

typedef struct KF_Vector_s {
	int16_t aCol[1];
}KF_Vector_t;

typedef struct KF_Matrix_s {
	int16_t aRow[DIMENSION];
//	uint8_t dim;
}KF_Matrix_t;

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
/* static StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_); */



/*=================================== >> GLOBAL VARIABLES << =====================================*/
// static KF_Vector_t columns[] = {
//		 {1000},
//		 {100},
// };
//
//static KF_Matrix_t matrix[] = INIT_MATRIX_DIM_2;

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
/* static StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_)
   { 
  	  // Write your code here!
   }
 */



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void KF_Init(void);
void KF_Main(void);


#ifdef MASTER_KF_C_
#undef MASTER_KF_C_
#endif /* !MASTER_KF_C_ */
