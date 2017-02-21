/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	03.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file sh_cfg.c
 * 
 *==================================================================================================
 */

#define MASTER_sh_cfg_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "task_clshdlr.h"
#include "appl_clshdlr.h"
#include "sh_clshdlr.h"
#include "rnet_clshdlr.h"

#include "sh_cfg.h"
#include "RTT1.h"

#include "buz.h"
#include "Motor.h"
#include "RNET1.h"
#include "LED1.h"
#include "LED2.h"
#include "batt.h"
#include "Q4CLeft.h"
#include "Q4CRight.h"
#include "Tacho.h"
#include "KIN1.h"
#include "Pid.h"
#include "drv.h"
#include "id.h"
#include "nvm.h"

#include "FRTOS1.h"



/*======================================= >> #DEFINES << =========================================*/
#define NUM_OF_IOS 	(sizeof(ios)/sizeof(ios[0]))

/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
static const CLS1_ParseCommandCallback CmdParserTable[] =
{
  CLS1_ParseCommand,
  SH_ParseCommand,
  FRTOS1_ParseCommand,
  TASK_ParseCommand,
  APPL_ParseCommand,
  NVM_ParseCommand,
  MOT_ParseCommand,
  DRV_ParseCommand,
  TACHO_ParseCommand,
  PID_ParseCommand,
  Q4CLeft_ParseCommand,
  Q4CRight_ParseCommand,
  BUZ_ParseCommand,
  LED1_ParseCommand,
  LED2_ParseCommand,
  RNET1_ParseCommand,
  BATT_ParseCommand,
  KIN1_ParseCommand,
  RNET_ParseCommand,
  NULL /* Sentinel */
};

static const SH_IODesc_t ios[] =
{
    {CLS1_DefaultShellBuffer, sizeof(CLS1_DefaultShellBuffer), &CLS1_stdio},
    {RTT1_DefaultShellBuffer, sizeof(RTT1_DefaultShellBuffer), &RTT1_stdio},
};

static const SH_IOCfg_t ioCfg =
{
		ios,
		NUM_OF_IOS,
};

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/
const CLS1_ParseCommandCallback *Get_CmdParserTbl() { return CmdParserTable; }

const SH_IOCfg_t *Get_ShIOCfg() { return &ioCfg; }

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/




#ifdef MASTER_sh_cfg_C_
#undef MASTER_sh_cfg_C_
#endif /* !MASTER_sh_cfg_C_ */
