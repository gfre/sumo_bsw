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



/*======================================= >> #DEFINES << =========================================*/
#define MOTOR_MAX_VAL (0xFFFF)


/*=================================== >> TYPE DEFINITIONS << =====================================*/
/* typedef unit8 TmplType_t; */



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/
/* static StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_); */



/*=================================== >> GLOBAL VARIABLES << =====================================*/
static PID_Plant_t plantTbl[] =
{
		{PID_LEFT_MOTOR_SPEED,  FALSE, 2000,  80,  0, 100, 0, MOTOR_MAX_VAL, 0, 0, PID_Init2, PID_Deinit2, MAF_Get_Speed,             DRV_Get_LeftSpeedTargetVal},
		{PID_RIGHT_MOTOR_SPEED, FALSE, 2000,  80,  0, 100, 0, MOTOR_MAX_VAL, 0, 0, PID_Init2, PID_Deinit2, MAF_Get_Speed,             DRV_Get_RightSpeedTargetVal},
		{PID_LEFT_MOTOR_POS,    FALSE, 1000,   1, 50, 100, 0, MOTOR_MAX_VAL, 0, 0, PID_Init2, PID_Deinit2, TACHO_Get_CurrentPosition, DRV_Get_LeftPosTargetVal},
		{PID_RIGHT_MOTOR_POS,   FALSE, 1000,   1, 50, 100, 0, MOTOR_MAX_VAL, 0, 0, PID_Init2, PID_Deinit2, TACHO_Get_CurrentPosition, DRV_Get_RightPosTargetVal},

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
