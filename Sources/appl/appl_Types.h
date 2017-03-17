/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	08.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file appl_Types.h
 * 
 *==================================================================================================
 */


#ifndef APPL_TYPES_H_
#define APPL_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"

#ifdef MASTER_appl_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/
/*!
 * Command ID's of the actions in the state
 */
typedef enum APPL_Cmd_e
{
	 Run = 0x00		/*!< called in every cycle in the current state */
    ,Enter			/*!< called when entering the target state */
    ,Exit  			/*!< called when leaving the current state */
	,CmdCnt			/*!< count of valid commands */
	,noCmd			/*!< no or invalid command */
}APPL_Cmd_t;

typedef enum APPL_State_e
{
	 APPL_STATE_STARTUP = 0x00	/**< State during start up */
	,APPL_STATE_INIT			/**< State during initialization */
	,APPL_STATE_IDLE			/**< State for idle mode */
	,APPL_STATE_NORMAL			/**< State for normal mode */
	,APPL_STATE_DEBUG			/**< State for debug mode */
	,APPL_STATE_ERROR			/**< State for error mode */
	,APPL_STATE_NUM         	/**< Number of states/modes */
	,APPL_STATE_NONE		 	/**< No or invalid state */
} APPL_State_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief This function returns the address of the variable of the application state
 * @return pointer to the variable of the application state
 */
EXTERNAL_ APPL_State_t APPL_Get_NextState(void);
EXTERNAL_ APPL_State_t APPL_Get_SmState(void);
EXTERNAL_ APPL_Cmd_t APPL_Get_SmCmd(void);

EXTERNAL_ StdRtn_t Set_HoldOnEnter(const APPL_State_t state_, const uint8_t holdOn_);
EXTERNAL_ StdRtn_t Set_HoldOnExit(const APPL_State_t state_, const uint8_t holdOn_);
EXTERNAL_ StdRtn_t Set_ReleaseEnter(const APPL_State_t state_);
EXTERNAL_ StdRtn_t Set_ReleaseExit(const APPL_State_t state_);

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !APPL_TYPES_H_ */
