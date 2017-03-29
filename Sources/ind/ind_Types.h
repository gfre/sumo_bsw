/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	29.03.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file ind_Types.h
 * 
 *==================================================================================================
 */


#ifndef IND_TYPES_H_
#define IND_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"

#ifdef MASTER_ind_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Function sets LED1 to flashing with certain period
 * @param perMS_ flashing period in milliseconds
 * @return Error code, ERR_OK if timer handle for LED1 is fine
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t IND_Flash_LED1WithPerMS(const uint16_t perMS_);

/**
 * @brief Function sets LED2 to flashing with certain period
 * @param perMS_ flashing period in milliseconds
 * @return Error code, ERR_OK if timer handle for LED2 is fine
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t IND_Flash_LED2WithPerMS(const uint16_t perMS_);

/**
 * @brief Function turns LED1 ON
 * @return Error code, ERR_OK if timer handle for LED1 is fine
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t IND_Set_LED1On();

/**
 * @brief Function turns LED2 ON
 * @return Error code, ERR_OK if timer handle for LED2 is fine
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t IND_Set_LED2On();

/**
 * @brief Function turns LED1 OFF
 * @return Error code, ERR_OK if timer handle for LED1 is fine
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t IND_Set_LED1Off();

/**
 * @brief Function turns LED2 OFF
 * @return Error code, ERR_OK if timer handle for LED2 is fine
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t IND_Set_LED2Off();

/**
 * @brief Function toggles LED1
 * @return Error code, ERR_OK if timer handle for LED1 is fine
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t IND_Set_LED1Toggle();

/**
 * @brief Function toggles LED2
 * @return Error code, ERR_OK if timer handle for LED2 is fine
 *                     ERR_PARAM_ADDRESS otherwise
 */
EXTERNAL_ StdRtn_t IND_Set_LED2Toggle();

/**
 * @brief Function reads the state of the LED1
 * @return state_ state of LED1
 */
EXTERNAL_ uint8_t IND_Get_LED1St();

/**
 * @brief Function reads the state of the LED2
 * @return state_ state of LED2
 */
EXTERNAL_ uint8_t IND_Get_LED2St();

#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !IND_TYPES_H_ */
