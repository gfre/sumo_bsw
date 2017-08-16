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

/*======================================= >> #DEFINES << =========================================*/
#define MOTOR_MAX_VAL        (0xFFFFu)
#define PID_LFT_MTR_SPD_STR  ("speed L")
#define PID_RGHT_MTR_SPD_STR ("speed R")
#define PID_LFT_MTR_POS_STR  ("pos L")
#define PID_RGHT_MTR_POS_STR ("pos R")
#define PID_LFT_TL_STR		 ("tl L")
#define PID_RGHT_TL_STR		 ("tl R")



/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/

/*=================================== >> GLOBAL VARIABLES << =====================================*/
							/* KP   |  KI  |  KD  |  Scale  |  PI Output Limit  */
static PID_Cfg_t LftSpdCfg  = {2000u,  80u,   0u,    100u,    MOTOR_MAX_VAL,};
static PID_Cfg_t RghtSpdCfg = {2000u,  80u,   0u,    100u,    MOTOR_MAX_VAL,};
static PID_Cfg_t LftPosCfg  = {1000u,  1u,    50u,   100u, 	  MOTOR_MAX_VAL,};
static PID_Cfg_t RghtPosCfg = {1000u,  1u,    50u,   100u, 	  MOTOR_MAX_VAL,};
static PID_Cfg_t TLLftCfg  	= {1000u,  10u,   0u,    100u, 	  MOTOR_MAX_VAL,}; //output of PI is velocity so 'PI Output Limit' value should be at least >5500 steps/s!
static PID_Cfg_t TLRghtCfg  = {1000u,  10u,   0u,    100u, 	  MOTOR_MAX_VAL,};

static PID_Plnt_t plntTbl[] =
{
		{PID_LFT_MTR_SPD_STR,  PID_LFT_MTR_SPD,  &LftSpdCfg,  PID_NO_SAT, 0, 0, TACHO_Read_CurFltrdLftSpd,  DRV_Read_LftSpdTrgtVal},
		{PID_RGHT_MTR_SPD_STR, PID_RGHT_MTR_SPD, &RghtSpdCfg, PID_NO_SAT, 0, 0, TACHO_Read_CurFltrdRghtSpd, DRV_Read_RghtSpdTrgtVal},
		{PID_LFT_MTR_POS_STR,  PID_LFT_MTR_POS,  &LftPosCfg,  PID_NO_SAT, 0, 0, TACHO_Read_CurLftPos,       DRV_Read_LftPosTrgtVal},
		{PID_RGHT_MTR_POS_STR, PID_RGHT_MTR_POS, &RghtPosCfg, PID_NO_SAT, 0, 0, TACHO_Read_CurRghtPos,	    DRV_Read_RghtPosTrgtVal},
		{PID_LFT_TL_STR,       PID_LFT_TL,		 &TLLftCfg,   PID_NO_SAT, 0, 0, TL_Read_CurLftPos,			TACHO_Read_CurLftPos,},
		{PID_RGHT_TL_STR,      PID_RGHT_TL,		 &TLRghtCfg,  PID_NO_SAT, 0, 0, TL_Read_CurRghtPos,		    TACHO_Read_CurRghtPos,},
};

static PID_PlntCfg_t pidCfg =
{
	plntTbl,
	sizeof(plntTbl)/sizeof(plntTbl[0]),
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/


/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
PID_PlntCfg_t* Get_pPidCfg(void) {return &pidCfg;}



#ifdef MASTER_pid_cfg_C_
#undef MASTER_pid_cfg_C_
#endif /* !MASTER_pid_cfg_C_ */
