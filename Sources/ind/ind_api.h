/***********************************************************************************************//**
 * @file		ind_api.h
 * @ingroup		ind
 * @brief 		API of the SWC @a Indication
 *
 * This API provides a BSW-internal interface of the SWC @ref ind. It is supposed to be available
 * to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	29.03.2017
 *  
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef IND_API_H_
#define IND_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "ACon_Types.h"



#ifdef MASTER_ind_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup ind
 * @{
 */
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



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !IND_API_H_ */
