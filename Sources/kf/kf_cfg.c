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
static const Matrix KF_A[] = //system matrix
{
		{1, 0.08},
		{0, 1},
};

static const Matrix KF_IdentityMatrix[] =
{
		{1, 0},
		{0, 1},
};

static const Matrix KF_H[] = //measurement Matrix
{
		{1, 0},
		{0, 1},
};

static const Matrix KF_R[] = { // measurement noise covariance matrix for v = 1000 (from matlab)
		{132.090106368499,	-3.69223227144139},
		{-3.69223227144139,	0.880407771722165},
};

static const Matrix KF_P_Initial_Error[] = { // initial error covariance matrix
		{2500,	-5000},
		{-5000,	10000},
};

static const Vector KF_InitialValues[] = // s = 0 steps, v = 1000 steps/s
{
		{0},
		{1000}, //steps/sec
};

static const Vector KF_x_Initial_Estimate[] =  { //initial estimate first time
		{50},  //position
		{900}, //velocity
 };

static const KF_Cfg_t kfCfg =
{
		&KF_A,
		&KF_H,
		&KF_IdentityMatrix,
		&KF_K_k,
		&KF_R,
		&KF_P_Initial_Error,
		&KF_InitialValues,
		&KF_x_Initial_Estimate,
};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
KF_Cfg_t GetKFCfg(void){
	return &kfCfg;
}
#ifdef MASTER_KF_CFG_C_
#undef MASTER_KF_CFG_C_
#endif /* !MASTER_KF_CFG_C_ */
