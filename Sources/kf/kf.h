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

#define MASTER_kf_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "PE_Types.h"

/*======================================= >> #DEFINES << =========================================*/
/* #define TMPL_MACRO (0x01u) */



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef struct {
	double column[2];
}Matrix;

typedef struct {
	double column[1];
}Vector;



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
/* static StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_); */



/*=================================== >> GLOBAL VARIABLES << =====================================*/
/* static TmplType_t tmplArray[STUD_MACRO] = {0u,TRUE,FALSE}; */



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
/* static StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_)
   { 
  	  // Write your code here!
   }
 */



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
void KF_Init(void);


#ifdef MASTER_kf_C_
#undef MASTER_kf_C_
#endif /* !MASTER_kf_C_ */
