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


#ifdef MASTER_appl_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/


/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum APPL_State_e
{
	 APPL_STATE_NONE = 0x00 	/**< No or invalid state */
	,APPL_STATE_STARTUP			/**< State during start up */
	,APPL_STATE_INIT			/**< State during initialization */
	,APPL_STATE_IDLE			/**< State for idle mode */
	,APPL_STATE_NORMAL			/**< State for normal mode */
	,APPL_STATE_DEBUG			/**< State for debug mode */
	,APPL_STATE_ERROR			/**< State for error mode */
	,APPL_STATE_NUM         	/**< Number of vaild states/modes */
} APPL_State_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/*!
 * @brief This function returns the address of the variable of the application state
 * @return pointer to the variable of the application state
 */
EXTERNAL_ APPL_State_t APPL_Get_State(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !APPL_TYPES_H_ */
