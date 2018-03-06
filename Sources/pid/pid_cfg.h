/***********************************************************************************************//**
 * @file		pid_cfg.h
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	<I>. <Surname>, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.08.2017
 *
 * @copyright @<LGPLx_x>
 *
 ***************************************************************************************************/

#ifndef PID_CFG_H_
#define PID_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "pid_api.h"



#ifdef MASTER_pid_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup pid
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 *
 * @param
 * @return
 */
typedef StdRtn_t PID_NVMReadFct_t(NVM_PidCfg_t*);

/**
 *
 * @param
 * @return
 */
typedef StdRtn_t PID_NVMSaveFct_t(const NVM_PidCfg_t *);

/**
 *
 */
typedef struct PID_NVM_s{
	PID_NVMReadFct_t *readFct;
	PID_NVMReadFct_t *readDfltFct;
	PID_NVMSaveFct_t *saveFct;
} PID_NVM_t;

/**
 *
 */
typedef struct PID_Cfg_s
{
	const uchar_t *pItmName;
	PID_Gain_t gain;
	PID_NVM_t nvm;
}PID_Cfg_t;

/**
 *
 */
typedef struct PID_Itm_s
{
	PID_Cfg_t cfg;
	PID_Data_t data;
}PID_Itm_t;

/**
 *
 */
typedef struct PID_ItmTbl_s
{
	PID_Itm_t    *aPids;
	uint8_t       numPids;
}PID_ItmTbl_t;

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ PID_ItmTbl_t *Get_pPidItmTbl(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !PID_CFG_H_ */
