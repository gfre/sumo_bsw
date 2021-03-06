/***********************************************************************************************//**
 * @file		refl.h
 * @ingroup		refl
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	G. Freudenthaler, gefr@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	08.06.2017
 *
 * @copyright @<LGPL2_1>
 *
 ***************************************************************************************************/

#ifndef REFL_H_
#define REFL_H_

/*======================================= >> #INCLUDES << ========================================*/

#ifdef MASTER_refl_C_
#define EXTERNAL_
#else
#define EXTERNAL_ extern
#endif

/*======================================= >> #DEFINES << =========================================*/
/**
 * String identification of the SWC @ref refl
 */
#define REFL_SWC_STRING ("refl")



/*=================================== >> TYPE DEFINITIONS << =====================================*/



/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/
/**
 * @brief Driver initialisation
 */
EXTERNAL_ void REFL_Init(const void *pvPar_);

/**
 * @brief Main function of the SWC @ref refl
 */
EXTERNAL_ void REFL_MainFct(void);



#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !SOURCES_REFL_H_ */
