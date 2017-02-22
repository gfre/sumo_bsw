/***************************************************************************************************
 * @brief 	>>TODO This is a brief description.
 *
 * @author 	>>TODO, gefr@tf.uni-kiel.de, University Kiel 
 * @date 	22.02.2017
 *  
 * @copyright 	LGPL-2.1, https://opensource.org/licenses/LGPL-2.1
 *
 * >>TODO This is the detailed description of the file id_Types.h
 * 
 *==================================================================================================
 */

#ifndef ID_TYPES_H_
#define ID_TYPES_H_

/*======================================= >> #INCLUDES << ========================================*/
#include "Platform.h"


#ifdef MASTER_id_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/



/*=================================== >> TYPE DEFINITIONS << =====================================*/
typedef enum ID_Sumo_e { /* do *not* change order of enumeration, they are used internally for a table index */
	 ID_SUMO_MIN  = 0x00					/**< min custom ID of sumo */
	,ID_SUMO_MAX  = MAX_ID_OF_SUMOS         /**< max custom ID of sumo */
	,ID_SUMO_UNKNOWN       					/**< unknown robot, unknown custom ID */
	,ID_SUMO_NONE          					/**< initialization value, used internally */
} ID_Sumo_t;


/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief This function returns the custom sumo ID
 * @return custom sumo ID
 */
EXTERNAL_ ID_Sumo_t Get_SumoID(void);


#ifdef EXTERNAL_
#undef EXTERNAL_
#endif


#endif /* !ID_TYPES_H_ */
