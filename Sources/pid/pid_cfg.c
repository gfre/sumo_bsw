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
#define MOTOR_MAX_VAL (0xFFFFu)
#define PID_LEFT_MOTOR_SPD_NAME  ("speed L")
#define PID_RIGHT_MOTOR_SPD_NAME ("speed R")
#define PID_LEFT_MOTOR_POS_NAME  ("pos L")
#define PID_RIGHT_MOTOR_POS_NAME ("pos R")
#define PID_LFT_TL_NAME		 	 ("tl L")
#define PID_RGHT_TL_NAME		 ("tl R")



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/* typedef unit8 TmplType_t; */



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
/* static StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_); */



/*=================================== >> GLOBAL VARIABLES << =====================================*/
							   /* KP   |  KI  |  KD  |  Scale  |  PI Output Limit  */
static PID_Cfg_t LeftSpeedCfg  = {2000u,  80u,   0u,    100u,     MOTOR_MAX_VAL,};
static PID_Cfg_t RightSpeedCfg = {2000u,  80u,   0u,    100u,     MOTOR_MAX_VAL,};
static PID_Cfg_t LeftPosCfg    = {1000u,  1u,    50u,   100u, 	  MOTOR_MAX_VAL,};
static PID_Cfg_t RightPosCfg   = {1000u,  1u,    50u,   100u, 	  MOTOR_MAX_VAL,};
static PID_Cfg_t tlLftCfg  	   = {1000u,  10u,   0u,    100u, 	  MOTOR_MAX_VAL,};
static PID_Cfg_t tlRghtCfg     = {1000u,  10u,   0u,    100u, 	  MOTOR_MAX_VAL,};



static PID_Plant_t plantTbl[] =
{
		{PID_LEFT_MOTOR_SPD_NAME,  PID_LEFT_MOTOR_SPEED,  &LeftSpeedCfg,  PID_NO_SAT, 0, 0, TACHO_Read_CurFltrdLeftSpd,  DRV_Read_LeftSpdTrgtVal},
		{PID_RIGHT_MOTOR_SPD_NAME, PID_RIGHT_MOTOR_SPEED, &RightSpeedCfg, PID_NO_SAT, 0, 0, TACHO_Read_CurFltrdRightSpd, DRV_Read_RightSpdTrgtVal},
		{PID_LEFT_MOTOR_POS_NAME,  PID_LEFT_MOTOR_POS,    &LeftPosCfg,    PID_NO_SAT, 0, 0, TACHO_Read_CurLeftPos,       DRV_Read_LeftPosTrgtVal},
		{PID_RIGHT_MOTOR_POS_NAME, PID_RIGHT_MOTOR_POS,   &RightPosCfg,   PID_NO_SAT, 0, 0, TACHO_Read_CurRightPos,		 DRV_Read_RightPosTrgtVal},
		{PID_LFT_TL_NAME,          PID_LFT_TL,			  &tlLftCfg,	  PID_NO_SAT, 0, 0, TL_Read_CurLftPos,			 TACHO_Read_CurLeftPos,},
		{PID_RGHT_TL_NAME,         PID_RGHT_TL,			  &tlRghtCfg,	  PID_NO_SAT, 0, 0, TL_Read_CurRghtPos,		 TACHO_Read_CurRightPos,},
};

static PID_PlantCfg_t pidCfg =
{
	plantTbl,
	sizeof(plantTbl)/sizeof(plantTbl[0]),
};


/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
PID_PlantCfg_t* Get_pPidCfg(void) {return &pidCfg;}



#ifdef MASTER_pid_cfg_C_
#undef MASTER_pid_cfg_C_
#endif /* !MASTER_pid_cfg_C_ */
