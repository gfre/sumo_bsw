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


#ifdef MASTER_ind_Types_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
EXTERNAL_ StdRtn_t IND_Flash_LED1WithPerMS(const uint16_t perMS_);
EXTERNAL_ StdRtn_t IND_Flash_LED2WithPerMS(const uint16_t perMS_);

EXTERNAL_ StdRtn_t IND_Set_LED1On();
EXTERNAL_ StdRtn_t IND_Set_LED2On();

EXTERNAL_ StdRtn_t IND_Set_LED1Off();
EXTERNAL_ StdRtn_t IND_Set_LED2Off();

EXTERNAL_ StdRtn_t IND_Set_LED1Toggle();
EXTERNAL_ StdRtn_t IND_Set_LED2Toggle();


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !IND_TYPES_H_ */
