/***********************************************************************************************//**
 * @file		appl_api.h
 * @ingroup		appl
 * @brief 		API of the SWC @a Application
 *
 * This API provides the internal interface of the Basic Software from the SWC @a Application to the
 * all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.02.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright 	@LGPL2_1
 *
 ***************************************************************************************************/

#ifndef APPL_API_H_
#define APPL_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"



#ifdef MASTER_appl_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup appl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/
/**
 * @brief Command ID's of the actions in the state
 */
typedef enum APPL_Cmd_e
{
	 APPL_Cmd_Run = 0x00		/**< called in every cycle in the current state */
    ,APPL_Cmd_Enter				/**< called when entering the target state */
    ,APPL_Cmd_Exit 				/**< called when leaving the current state */
	,APPL_Cmd_Cnt				/**< count of valid commands */
	,APPL_Cmd_None				/**< no or invalid command */
}APPL_Cmd_t;


/**
 * @brief ID's of Application states
 */
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
 * @brief This function returns the next targeted state of state machine handling application software
 * @return targeted state
 */
EXTERNAL_ APPL_State_t APPL_Get_NextState(void);

/*!
 * @brief This function returns the current state of state machine handling application software
 * @return current application state
 */
EXTERNAL_ APPL_State_t APPL_Get_SmState(void);

/*!
 * @brief This function returns the current command of state machine handling application software
 * @return current application state
 */
EXTERNAL_ APPL_Cmd_t APPL_Get_SmCmd(void);

/**
 * @brief This function re-initialises the application. The application state machine re-starts in INIT-state.
 */
EXTERNAL_ void APPL_Set_ReInitAppl(void);

/**
 * @brief This function triggers the transition from IDLE state to NORMAL state.
 * @return Error code, - ERR_OK if everything was fine,\n
 * 					   - ERR_PARAM_CONDITION if calling state was not IDLE.
 */
EXTERNAL_ StdRtn_t APPL_Set_TransIdle2Normal(void);

/**
 * @brief This function sets a flag which enables/disables hold on ENTER functionality for a certain state
 * @param state_ corresponding application state for holdOn ENTER functionality
 * @param holdOn_ TRUE/FALSE-flag for enabling or disabling holdOn ENTER functionality
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t Set_HoldOnEnter(const APPL_State_t state_, const uint8_t holdOn_);

/**
 * @brief This function sets a flag which enables/disables hold on EXIT functionality for a certain state
 * @param state_ corresponding application state for holdOn EXIT functionality
 * @param holdOn_ TRUE/FALSE-flag for enabling or disabling holdOn EXIT functionality
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t Set_HoldOnExit(const APPL_State_t state_, const uint8_t holdOn_);

/**
 * @brief This function allows to release hold on ENTER for a certain state during runtime
 * @param state_ corresponding application state which shall be released
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t Set_ReleaseEnter(const APPL_State_t state_);

/**
 * @brief This function allows to release hold on EXIT for a certain state during runtime
 * @param state_ corresponding application state which shall be released
 * @return Error code, ERR_OK if everything was fine,
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t Set_ReleaseExit(const APPL_State_t state_);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !APPL_API_H_ */
