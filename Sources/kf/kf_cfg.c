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
#include "tacho.h"

/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static KF_I32Mat_t KF_A =
{
		{
				{1*DIVIDER,((TACHO_SAMPLE_PERIOD_MS*DIVIDER)/1000)}, 		// s_k_hat = s_k-1_hat [steps] + 5 ms * v_k-1_hat [steps/s] -> [steps] + [milli steps] <- this value divided by 1000 gives steps
				{0, 1*DIVIDER},
		},
};

static KF_I32Mat_t KF_Identity_Matrix =
{
		{
				{10000, 0},
				{0, 10000},
		},
};

static KF_I32RowVec_t KF_c_Transposed = //measurement Vector
{
				{1, 0},
};

static int32_t KF_r = 20;// measurement noise covariance for s


static KF_I32Mat_t KF_Q  = // process noise covariance matrix
{
		{
				{16, 0},
				{0, 1600},
		},
};

static KF_I32Mat_t KF_P_Initial_Error = // initial error in estimate covariance matrix
{
		{
			{100, 0},
			{0, 100},
		},
};

static KF_I32ColVec_t KF_x_Initial_Estimate =  //initial estimate
{
		{930, 120},
};

static KF_Cfg_t kfCfg =
{
		&KF_A,
		&KF_c_Transposed,
		&KF_Identity_Matrix,
		&KF_r,
		&KF_Q,
		&KF_P_Initial_Error,
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
