/***********************************************************************************************//**
 * @file		kf_cfg.c
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	<I>. <Surname>, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	26.06.2017
 *
 * @copyright @<LGPLx_x>
 *
 ***************************************************************************************************/

#define MASTER_KF_CFG_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "kf.h"
#include "kf_cfg.h"

/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static KF_Matrix_t KF_A[] = //system matrix
{
		{1, 80}, 		// s_hat_k = s_hat_k-1 [steps] + 80 ms * v_hat_k-1 [steps/s] -> [steps] + [milli steps] <- this value divided by 1000 gives steps
		{0, 1},
};

static KF_Matrix_t KF_Identity_Matrix[] =
{
		{1, 0},
		{0, 1},
};

static KF_Matrix_t KF_H[] = //measurement Matrix
{
		{1, 0},
		{0, 1},
};

static KF_Matrix_t KF_R[] = { // measurement noise covariance matrix for v = 1000 (from matlab)
		{0.880407771722165,	-3.69223227144139},
		{-3.69223227144139,	132.090106368499},
};

static KF_Matrix_t KF_Q[] = { // process noise covariance matrix for v = 1000 (from matlab)
		{1, 0},
		{0, 1},
};

static KF_Matrix_t KF_P_Initial_Error[] = { // initial error in estimate covariance matrix
		{10, 0},		//{2500,	-5000},
		{0,	10},		// -5000 10000
};

static KF_Vector_t KF_x_Initial_Values[] = // s = 0 steps, v = 1000 steps/s
{
		{0},
		{0}, //steps/sec
};

static KF_Vector_t KF_x_Initial_Estimate[] =  { //initial estimate first time
		{10},  //position in steps
		{20}, //velocity in steps/s
 };

static KF_Cfg_t kfCfg =
{
		KF_A,
		KF_H,
		KF_Identity_Matrix,
		KF_R,
		KF_Q,
		KF_P_Initial_Error,
		KF_x_Initial_Values,
		KF_x_Initial_Estimate,
};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
const KF_Cfg_t* GetKFCfg(void){
	return &kfCfg;
}
#ifdef MASTER_KF_CFG_C_
#undef MASTER_KF_CFG_C_
#endif /* !MASTER_KF_CFG_C_ */
