/***********************************************************************************************//**
 * @file		id_api.h
 * @ingroup		id
 * @brief 		API of the SWC @a Identification
 *
 * This API provides a BSW-internal interface of the SWC @ref id. It is supposed to be available
 * to all other Basic Software Components.
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	22.02.2017
 *
 * @note API for BSW-internal use only
 *
 * @copyright @LGPL2_1
 *
 ***************************************************************************************************/

#ifndef ID_API_H_
#define ID_API_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "rte_Types.h"


#ifdef MASTER_id_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/**
 * @addtogroup id
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This function returns the custom sumo ID
 * @return custom sumo ID
 */
EXTERNAL_ ID_Sumo_t Get_SumoID(void);



/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !ID_API_H_ */
