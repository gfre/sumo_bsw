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
static KF_I32Mat_t KF_A = //system matrix
{
		{
				{1000, 80}, 		// s_hat_k = s_hat_k-1 [steps] + 80 ms * v_hat_k-1 [steps/s] -> [steps] + [milli steps] <- this value divided by 1000 gives steps
				{0, 1000},
		},
};

static KF_I32Mat_t KF_Identity_Matrix =
{
		{
				{1, 0},
				{0, 1},
		},
};

static KF_I32RowVec_t KF_c_Transposed = //measurement Vector
{
				{1, 0},
};

static int32_t KF_r = 1;// measurement noise covariance for s


static KF_I32Mat_t KF_Q = // process noise covariance matrix for v = 1000 (from matlab)
{
		{
				{1, 0},
				{0, 1},
		},
};

static KF_I32Mat_t KF_P_Initial_Error = // initial error in estimate covariance matrix
{
		{
			{1000, 0},		//{2500,	-5000},
			{0,	1000},		// -5000 10000
		},
};

static KF_I32ColVec_t KF_x_Initial_Values = // s = 0 steps, v = 1000 steps/s
{
		{0,0}, //steps/sec
};

static KF_I32ColVec_t KF_x_Initial_Estimate =  //initial estimate first time
{
		{10, 20},
};

static KF_Cfg_t kfCfg =
{
		&KF_A,
		&KF_c_Transposed,
		&KF_Identity_Matrix,
		&KF_r,
		&KF_Q,
		&KF_P_Initial_Error,
		&KF_x_Initial_Values,
		&KF_x_Initial_Estimate,
};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
const KF_Cfg_t* GetKFCfg(void){
	return &kfCfg;
}
#ifdef MASTER_KF_CFG_C_
#undef MASTER_KF_CFG_C_
#endif /* !MASTER_KF_CFG_C_ */
