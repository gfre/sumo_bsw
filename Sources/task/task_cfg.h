/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file task_cfg.h
 * 
 *==================================================================================================
 */


#ifndef TASK_CFG_H_
#define TASK_CFG_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "task_Types.h"


#ifdef MASTER_task_cfg_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
#define APPL_TASK_STRING  ("APPL")
#define COMM_TASK_STRING  ("COMM")
#define SH_TASK_STRING    ("SHELL")
#define DRV_TASK_STRING   ("DRIVE")



/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
  @brief interface function

  Here is a brief description of the interface function.
  @param *output_ this is an ouput parameter
  @param *input_  this is an input parameter
*/
EXTERNAL_ const TASK_Cfg_t *TASK_Get_TasksCfg(void);

EXTERNAL_ const TASK_CfgItm_t *TASK_Get_ApplTaskCfg(void);

EXTERNAL_ const TASK_CfgItm_t *TASK_Get_ShTaskCfg(void);



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/* EXTERNAL_ StdRtn_t TMPL_CustomFct(const TmplType_t *input_, TmplType_t *output_); */


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !TASK_CFG_H_ */
