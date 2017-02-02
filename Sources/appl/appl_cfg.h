/*******************************************************************************
 * @brief 	This module configures the task creation
 *
 * @author 	Gerhard Freudenthaler, gefr@tf.uni-kiel.de, CAU Kiel 
 * @date 	12.01.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * This is the detailed description of the file CAU_Types.h
 * 
 * ==============================================================================
 */

#ifndef APPL_CFG_H_
#define APPL_CFG_H_

#include "appl_Types.h"

#ifdef MASTER_APPL_CFG_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif


#define SH_TASK_STRING "SHELL"

/**
  @brief interface function

  Here is a brief description of the interface function.
  @param *output_ this is an ouput parameter
  @param *input_  this is an input parameter
*/
EXTERNAL_ const APPL_TaskCfg_t *Get_APPL_TaskCfg(void);

EXTERNAL_ const APPL_TaskCfgItm_t *Get_APPL_MainTaskCfg(void);
EXTERNAL_ const APPL_TaskCfgItm_t *Get_APPL_ShTaskCfg(void);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* APPL_CFG_H */
