/***********************************************************************************************//**
 * @file		pid_cfg.c
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	G. Freudenthaler, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @author 	S. Helling, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	11.08.2017
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#define MASTER_pid_cfg_C_

// TODO
// - rework scaling of gains from decimal point to binary point
// - Add #ID in error message in cls handler
// - Implement PID for ramps/trajectories
// - Move strings to parent component DRV

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
static PID_Itm_t items[] =
{
		{	{PID_LFT_MTR_SPD_STR,  {2000u, 80u, 0u, 100u, MOTOR_MAX_VAL},
			{NVM_Read_PIDSpdLeCfg, NVM_Read_Dflt_PIDSpdLeCfg, NVM_Save_PIDSpdLeCfg}},
			{0}
		},
		{	{PID_RGHT_MTR_SPD_STR, {2000u, 80u, 0u, 100u, MOTOR_MAX_VAL},
			{NVM_Read_PIDSpdRiCfg, NVM_Read_Dflt_PIDSpdRiCfg, NVM_Save_PIDSpdRiCfg}},
			{0}
		},
		{ 	{PID_LFT_MTR_POS_STR,  {1000u, 1u, 50u, 100u, MOTOR_MAX_VAL},
			{NVM_Read_PIDPosCfg, NVM_Read_Dflt_PIDPosCfg, NVM_Save_PIDPosCfg}},
			{0}
		},
		{ 	{PID_RGHT_MTR_POS_STR, {1000u, 1u, 50u, 100u, MOTOR_MAX_VAL},
			{NVM_Read_PIDPosCfg, NVM_Read_Dflt_PIDPosCfg, NVM_Save_PIDPosCfg}},
			{0}
		},
};

static PID_ItmTbl_t itemTable =
{
	items,
	sizeof(items)/sizeof(items[0]),
};



/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/



/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/
PID_ItmTbl_t *Get_pPidItmTbl(void) {return &itemTable;}



#ifdef MASTER_pid_cfg_C_
#undef MASTER_pid_cfg_C_
#endif /* !MASTER_pid_cfg_C_ */
