/***********************************************************************************************//**
 * @file		pid_cfg.c
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

#define MASTER_pid_cfg_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "pid_cfg.h"
#include "pid_api.h"
#include "drv_api.h"
#include "tacho_api.h"
#include "maf_api.h"
#include "tl_api.h"
#include "nvm_api.h"

/*======================================= >> #DEFINES << =========================================*/
#define MOTOR_MAX_VAL        (0xFFFFu)
#define PID_LFT_MTR_SPD_STR  ("speed L")
#define PID_RGHT_MTR_SPD_STR ("speed R")
#define PID_LFT_MTR_POS_STR  ("pos L")
#define PID_RGHT_MTR_POS_STR ("pos R")



/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/
							   /* KP   |  KI  |  KD  |  Scale  |  PI Output Limit  */
static PID_PrmCfg_t LftSpdCfg  = {2000u,  80u,   0u,    100u,    MOTOR_MAX_VAL,};
static PID_PrmCfg_t RghtSpdCfg = {2000u,  80u,   0u,    100u,    MOTOR_MAX_VAL,};
static PID_PrmCfg_t LftPosCfg  = {1000u,  1u,    50u,   100u, 	 MOTOR_MAX_VAL,};
static PID_PrmCfg_t RghtPosCfg = {1000u,  1u,    50u,   100u,    MOTOR_MAX_VAL,};

static PID_Itm_t itmTbl[] =
{
		{PID_LFT_MTR_SPD_STR,  PID_LFT_MTR_SPD,  &LftSpdCfg,  PID_NO_SAT, 0, 0, TACHO_Read_CurFltrdLftSpd,  DRV_Read_LftSpdTrgtVal,  NVM_Read_PIDSpdLeCfg, NVM_Read_Dflt_PIDSpdLeCfg, NVM_Save_PIDSpdLeCfg},
		{PID_RGHT_MTR_SPD_STR, PID_RGHT_MTR_SPD, &RghtSpdCfg, PID_NO_SAT, 0, 0, TACHO_Read_CurFltrdRghtSpd, DRV_Read_RghtSpdTrgtVal, NVM_Read_PIDSpdRiCfg, NVM_Read_Dflt_PIDSpdRiCfg, NVM_Save_PIDSpdRiCfg},
		{PID_LFT_MTR_POS_STR,  PID_LFT_MTR_POS,  &LftPosCfg,  PID_NO_SAT, 0, 0, TACHO_Read_CurLftPos,       DRV_Read_LftPosTrgtVal,  NVM_Read_PIDPosCfg,   NVM_Read_Dflt_PIDPosCfg,   NVM_Save_PIDPosCfg},
		{PID_RGHT_MTR_POS_STR, PID_RGHT_MTR_POS, &RghtPosCfg, PID_NO_SAT, 0, 0, TACHO_Read_CurRghtPos,	    DRV_Read_RghtPosTrgtVal, NVM_Read_PIDPosCfg,   NVM_Read_Dflt_PIDPosCfg,   NVM_Save_PIDPosCfg},
};

static PID_Cfg_t pidCfg =
{
	itmTbl,
	sizeof(itmTbl)/sizeof(itmTbl[0]),
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
PID_Cfg_t* Get_pPidCfg(void) {return &pidCfg;}


#ifdef MASTER_pid_cfg_C_
#undef MASTER_pid_cfg_C_
#endif /* !MASTER_pid_cfg_C_ */
