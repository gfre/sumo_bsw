/***********************************************************************************************//**
 * @file		  kf_cfg.h
 * @ingroup		kf Kalman Filter
 * @brief 		This header file contains the type definitions and type configurations for matrices
 * 				for the Kalman filter.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de,      Chair of Automatic Control, University Kiel
 * @author 	S. Helling,       stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	05.03.2018
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef KF_CFG_H
#define KF_CFG_H

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"
#include "Acon_Types.h"
#include "mtx_api.h"


#ifdef MASTER_KF_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


/*======================================= >> #DEFINES << =========================================*/
/**
 * @brief Defines the default maximum modulo value for state variables, must be less than 0x7FFF (32767)
 */
#define KF_DFLT_MAX_MOD_VAL (30000u)

/**
 * @brief Defines the initial/reset value for P0 = diag{alpha} (more precisely: UP0 = eye(dim), DP0 = diag{alpha})
 */
#define KF_DFLT_ALPHA (100u)


/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @brief Function pointer to a measurement/input function
 */
typedef StdRtn_t (*KF_ReadFct_t)(int32_t*);

/**
 * @brief Contains runtime data of a Kalman filter item
 */
typedef struct KF_Data_s
{
	MTX_t  vXapri;							/**< the a priori state estimate */
	MTX_t  vXapost;							/**< the a posteriori state estimate */
	MTX_t  mUPapri;							/**< the a priori unit upper triangular matrix */
	MTX_t  mDPapri;							/**< the a priori diagonal matrix matrix */
	MTX_t  mUPapost;						/**< the a priori unit upper triangular matrix */
	MTX_t  mDPapost;						/**< the a aposteriori diagonal matrix */
	int32_t aModCntr[FIXMATRIX_MAX_SIZE];	/**< modulo counter for state variables. Change FIXMATRIX_MAX_SIZE in compiler flags */
}KF_Data_t;

/**
 * @brief Defines the system's matrices/vectors
 */
typedef struct KF_MtxCfg_s
{
	 MTX_t mPhi;		/**< state transition matrix */
	 MTX_t mGamma;	/**< input matrix */
	 MTX_t mH;			/**< measurement matrix */
	 MTX_t mR;			/**< measurement noise covariance matrix (must be diagonal) */
	 MTX_t mG;			/**< process noise coupling matrix */
	 MTX_t mQ;			/**< process noise matrix (must be diagonal) */
}KF_MtxCfg_t;

/**
 * @brief Configuration information for a Kalman filter item
 */
typedef struct KF_Cfg_s
{
	const uchar_t *pItmName; 	  	/**< Kalman filter name */
	const uint8_t smplTimeMS;		  /**< Sample time */
	KF_MtxCfg_t   mtx;				    /**< System configuration */
	KF_ReadFct_t  *aMeasValFct;		/**< array of function pointers for measurements */
	KF_ReadFct_t  *aInptValFct;		/**< array of function pointers for inputs */
	bool bModCntrFlag;				    /**< flag to indicate usage of modulo counter for state variables */
}KF_Cfg_t;

/**
 * @brief Kalman filter item
 */
typedef struct KF_Itm_s
{
	KF_Cfg_t  cfg;		/**< configuration */
	KF_Data_t data;		/**< runtime data */
}KF_Itm_t;

/**
 * @brief Contains all Kalman filters
 */
typedef struct KF_ItmTbl_s
{
	KF_Itm_t *aKfs;			  /**< array of Kalman filter items */
	const uint8_t numKfs;	/**< total number of Kalman filters*/
}KF_ItmTbl_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Access function to Kalman filter item table
 * @return Pointer to the Kalman filter table
 */
EXTERNAL_ KF_ItmTbl_t *Get_pKfItmTbl(void);



/**
 * @brief Wrapper functions to fit data type for KF_ReadFct_t
 */
EXTERNAL_ StdRtn_t KF_Read_Rawi32SpdLe(int32_t *spd_);
EXTERNAL_ StdRtn_t KF_Read_Rawi32SpdRi(int32_t *spd_);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* KF_CFG_H */
