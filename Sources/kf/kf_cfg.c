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
#include "kf_cfg.h"
#include "tacho.h"

/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/
static KF_I32Mat_t KF_SystemMatrix =
{
		{
				{1*KF_SCALE_A, ((TACHO_SAMPLE_PERIOD_MS*KF_SCALE_A)/1000)},
				{0, 1*KF_SCALE_A},
		},
};

static KF_I32Mat_t KF_IdentityMatrix =
{
		{
				{KF_SCALE_KALMANGAIN, 0},
				{0, KF_SCALE_KALMANGAIN},
		},
};

static KF_I32RowVec_t KF_MeasurementVectorTransposed = //measurement Vector
{
				{1, 0},
};

static KF_I32Mat_t KF_MeasurementMatrix =
{
		{
				{1, 0},
				{0, 1},
		},
};

static int32_t KF_MeasurementNoiseCov = 1*KF_SCALE_ERROR;// measurement noise covariance for s

static KF_I32Mat_t KF_MeasNoiseCovMatrix =
{
		{
				{3*KF_SCALE_ERROR, 0},
				{0, 5*KF_SCALE_ERROR},
		},
};


static KF_I32Mat_t KF_ProcessNoiseCov = // process noise covariance matrix
{
		{
				{10*KF_SCALE_ERROR, 0},
				{0, 2500*KF_SCALE_ERROR},
		},
};

static KF_I32Mat_t KF_InitialErrorInEstimate = // initial error in estimate covariance matrix
{
		{
			{100, 0},
			{0, 100},
		},
};

static KF_I32ColVec_t KF_StateInitialEstimate =  //initial estimate
{
		{0, 0},
};

static KF_Cfg_t kfCfg =
{
		&KF_SystemMatrix,
		&KF_MeasurementVectorTransposed,
		&KF_MeasurementMatrix,
		&KF_IdentityMatrix,
		&KF_MeasurementNoiseCov,
		&KF_MeasNoiseCovMatrix,
		&KF_ProcessNoiseCov,
		&KF_InitialErrorInEstimate,
		&KF_StateInitialEstimate,
};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
const KF_Cfg_t* GetKFCfg(void){
	return &kfCfg;
}


#ifdef MASTER_KF_CFG_C_
#undef MASTER_KF_CFG_C_
#endif /* !MASTER_KF_CFG_C_ */
