/***********************************************************************************************//**
 * @file		refl.h
 * @ingroup		refl
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	Simon Helling, stu112498@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
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

/**
 * @addtogroup refl
 * @{
 */
/*======================================= >> #DEFINES << =========================================*/
/*/**
 * @brief
 *
 */


/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================ >> GLOBAL FUNCTION DECLARATIONS << ================================*/


/*!
 * \brief Driver Deinitialization.
 */
EXTERNAL_ void REF_Deinit(void);

/*!
 * \brief Driver Initialization.
 */
EXTERNAL_ void REF_Init(void);


/**
 * @}
 */
#ifdef EXTERNAL_
#undef EXTERNAL_
#endif

#endif /* !SOURCES_REFL_H_ */
